//! Rust version of Valhalla's [`valhalla_service`](https://github.com/valhalla/valhalla/blob/master/src/valhalla_service.cc)
//! that exposes the Actor API over HTTP.
//! Supports all [Valhalla endpoints](https://valhalla.github.io/valhalla/api/) (`/route`, `/sources_to_targets`, `/isochrone`, ...)
//! and both JSON and protobuf request and response formats.
//!
//! Requests may be sent as `POST` with a body (`application/json` or `application/x-protobuf`)
//! or as `GET` with a `?json=<url-encoded>` query parameter.
//!
//! Run with:
//!   cargo run -p valhalla-service --release -- path/to/valhalla.json --port 3000 --concurrency 8

use std::num::NonZero;

use axum::{
    Router,
    body::Body,
    extract::{Request, State},
    http::{Response, StatusCode, header},
    routing::post,
};
use clap::Parser;
use prost::Message;
use tokio::{signal, sync::oneshot};
use tracing::{info, warn};
use valhalla::proto::{self, options::Action};

/// Max request body size; bounds memory, large enough for `/trace_*` shape payloads.
const MAX_BODY_SIZE: usize = 32 * 1024 * 1024;

/// HTTP path to [`Action`]; also drives route registration.
const ROUTES: &[(&str, Action)] = &[
    ("/route", Action::Route),
    ("/locate", Action::Locate),
    ("/sources_to_targets", Action::SourcesToTargets),
    ("/optimized_route", Action::OptimizedRoute),
    ("/isochrone", Action::Isochrone),
    ("/trace_route", Action::TraceRoute),
    ("/trace_attributes", Action::TraceAttributes),
    ("/transit_available", Action::TransitAvailable),
    ("/expansion", Action::Expansion),
    ("/centroid", Action::Centroid),
    ("/status", Action::Status),
];

#[derive(Parser)]
struct Config {
    /// Path to valhalla json config file.
    valhalla_config: std::path::PathBuf,
    /// Port to listen
    #[arg(long, default_value_t = 3000)]
    port: u16,
    /// Number of worker threads / `Actor` instances. Defaults to the number of available CPUs.
    #[arg(long)]
    concurrency: Option<u16>,
}

type RequestMessage = (
    proto::Options,
    Action,
    oneshot::Sender<Result<valhalla::Response, valhalla::Error>>,
);

#[derive(Clone)]
struct AppState {
    /// Sends requests to the worker thread pool.
    workers: flume::Sender<RequestMessage>,
}

fn main() {
    tracing_subscriber::fmt::init();

    let config = Config::parse();

    let concurrency = config
        .concurrency
        .map(usize::from)
        .or(std::thread::available_parallelism().map(NonZero::get).ok())
        .unwrap_or(8) // fallback if CPU count is unknown
        .max(1);

    let valhalla_config = valhalla::Config::from_file(config.valhalla_config)
        .expect("Failed to open Valhalla config");

    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(concurrency.div_ceil(4)) // 1 tokio thread per 4 valhalla workers
        .enable_all()
        .build()
        .unwrap()
        .block_on(run(valhalla_config, config.port, concurrency))
}

async fn run(config: valhalla::Config, port: u16, concurrency: usize) {
    let app = build_app(&config, concurrency);

    let listener = tokio::net::TcpListener::bind(("0.0.0.0", port))
        .await
        .unwrap();
    info!("Listening at http://localhost:{port}");
    axum::serve(listener, app)
        .with_graceful_shutdown(async {
            tokio::select! {
                _ = signal::ctrl_c() => {
                    info!("Ctrl+C received, shutting down");
                }
                _ = async {
                    signal::unix::signal(signal::unix::SignalKind::terminate())
                        .expect("failed to install SIGTERM signal handler")
                        .recv()
                        .await
                } => {
                    info!("SIGTERM received, shutting down");
                }
            }
        })
        .await
        .unwrap();
}

/// Spawns `concurrency` worker threads (each with its own [`valhalla::Actor`]) and returns a
/// [`Router`] that load-balances across them. Workers stop once the router is dropped.
fn build_app(valhalla_config: &valhalla::Config, concurrency: usize) -> Router {
    let (worker_tx, worker_rx) = flume::bounded::<RequestMessage>(concurrency * 4);

    for worker_id in 0..concurrency {
        let mut actor =
            valhalla::Actor::new(valhalla_config).expect("Failed to create Valhalla actor");
        let rx = worker_rx.clone();

        std::thread::Builder::new()
            .name(format!("valhalla-worker-{worker_id}"))
            .spawn(move || {
                while let Ok((request, action, response_tx)) = rx.recv() {
                    let result = match action {
                        Action::Route => actor.route(&request),
                        Action::Locate => actor.locate(&request),
                        Action::SourcesToTargets => actor.matrix(&request),
                        Action::OptimizedRoute => actor.optimized_route(&request),
                        Action::Isochrone => actor.isochrone(&request),
                        Action::TraceRoute => actor.trace_route(&request),
                        Action::TraceAttributes => actor.trace_attributes(&request),
                        Action::TransitAvailable => actor.transit_available(&request),
                        Action::Expansion => actor.expansion(&request),
                        Action::Centroid => actor.centroid(&request),
                        Action::Status => actor.status(&request),
                        other => {
                            warn!("worker {worker_id} received unsupported action {other:?}");
                            continue;
                        }
                    };
                    let _ = response_tx.send(result);
                }
                info!("Valhalla worker {worker_id} shutting down");
            })
            .unwrap();
    }

    let state = AppState { workers: worker_tx };
    let mut app = Router::new();
    for &(path, action) in ROUTES {
        let handler = move |State(state): State<AppState>, request: Request| {
            handle_request(state, request, action)
        };
        app = app.route(path, post(handler).get(handler));
    }
    app.with_state(state)
}

async fn handle_request(state: AppState, request: Request, action: Action) -> Response<Body> {
    let valhalla_request = match parse_request(request, action).await {
        Ok(request) => request,
        Err((status, message)) => return error_response(status, message),
    };

    let (tx, rx) = oneshot::channel();
    if state
        .workers
        .send_async((valhalla_request, action, tx))
        .await
        .is_err()
    {
        return error_response(
            StatusCode::SERVICE_UNAVAILABLE,
            "no Valhalla workers available",
        );
    }

    match rx.await {
        Ok(result) => to_http_response(result),
        Err(_) => error_response(
            StatusCode::INTERNAL_SERVER_ERROR,
            "Valhalla worker dropped the request",
        ),
    }
}

/// Parses a request into [`proto::Options`] using Valhalla's precedence: protobuf body if
/// `application/x-protobuf`, else `?json=` query, else JSON body (`{}` if empty). Errors as an
/// HTTP status + message.
async fn parse_request(
    request: Request,
    action: Action,
) -> Result<proto::Options, (StatusCode, String)> {
    // GET carries the request in `?json=`; an empty value counts as absent.
    let json_query = request.uri().query().and_then(|query| {
        form_urlencoded::parse(query.as_bytes())
            .find(|(key, _)| key == "json")
            .map(|(_, value)| value.into_owned())
            .filter(|value| !value.is_empty())
    });

    // Content type matched case-insensitively, ignoring any `; charset=...` suffix.
    let is_protobuf = request
        .headers()
        .get(header::CONTENT_TYPE)
        .and_then(|value| value.to_str().ok())
        .and_then(|value| value.split(';').next())
        .is_some_and(|value| value.trim().eq_ignore_ascii_case("application/x-protobuf"));

    // Reject over-limit bodies up front when the length is declared (precise 413).
    if let Some(length) = request
        .headers()
        .get(header::CONTENT_LENGTH)
        .and_then(|value| value.to_str().ok())
        .and_then(|value| value.parse::<usize>().ok())
        && length > MAX_BODY_SIZE
    {
        return Err((
            StatusCode::PAYLOAD_TOO_LARGE,
            format!("body of {length} bytes exceeds the {MAX_BODY_SIZE} byte limit"),
        ));
    }

    let body = axum::body::to_bytes(request.into_body(), MAX_BODY_SIZE)
        .await
        .map_err(|err| (StatusCode::BAD_REQUEST, err.to_string()))?;

    if is_protobuf {
        return proto::Api::decode(body.as_ref())
            .map_err(|err| (StatusCode::BAD_REQUEST, err.to_string()))?
            .options
            .ok_or((
                StatusCode::BAD_REQUEST,
                "missing `options` field".to_owned(),
            ));
    }

    // JSON from `?json=` or the body; empty request means `{}` (valid for e.g. `/status`).
    let json = match &json_query {
        Some(json) => json.as_str(),
        None if body.is_empty() => "{}",
        None => {
            std::str::from_utf8(&body).map_err(|err| (StatusCode::BAD_REQUEST, err.to_string()))?
        }
    };
    valhalla::Actor::parse_json_request(json, action)
        .map_err(|err| (StatusCode::BAD_REQUEST, err.to_string()))
}

fn to_http_response(
    valhalla_response: Result<valhalla::Response, valhalla::Error>,
) -> Response<Body> {
    match valhalla_response {
        Ok(valhalla::Response::Pbf(api)) => Response::builder()
            .status(StatusCode::OK)
            .header(header::CONTENT_TYPE, "application/x-protobuf")
            .body(Body::from(api.encode_to_vec()))
            .unwrap(),
        Ok(valhalla::Response::Json(json)) => Response::builder()
            .status(StatusCode::OK)
            .header(header::CONTENT_TYPE, "application/json")
            .body(Body::from(json))
            .unwrap(),
        Ok(valhalla::Response::Other(bytes)) => Response::builder()
            .status(StatusCode::OK)
            .header(header::CONTENT_TYPE, "application/octet-stream")
            .body(Body::from(bytes))
            .unwrap(),
        Err(err) => error_response(StatusCode::BAD_REQUEST, err.to_string()),
    }
}

fn error_response(status: StatusCode, message: impl Into<Body>) -> Response<Body> {
    Response::builder()
        .status(status)
        .header(header::CONTENT_TYPE, "text/plain")
        .body(message.into())
        .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use tower::ServiceExt; // for `Router::oneshot`

    const TILES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tests/andorra/tiles.tar");
    // Sant Julia de Loria -> Andorra la Vella.
    const ROUTE_JSON: &str = r#"{"locations":[{"lat":42.50107,"lon":1.51034},{"lat":42.50627,"lon":1.52173}],"costing":"auto"}"#;

    /// Service backed by the Andorra test tiles; [`valhalla::ConfigBuilder`] fills defaults, so no
    /// JSON config file is needed.
    fn andorra_app() -> Router {
        let config = valhalla::ConfigBuilder {
            mjolnir: valhalla::config::Mjolnir {
                tile_extract: TILES.into(),
                ..Default::default()
            },
            ..Default::default()
        }
        .build();
        build_app(&config, 1)
    }

    /// Sends a request and returns the status, content type, and raw body.
    async fn send_full(app: &Router, request: Request) -> (StatusCode, String, Vec<u8>) {
        let response = app.clone().oneshot(request).await.unwrap();
        let status = response.status();
        let content_type = response
            .headers()
            .get(header::CONTENT_TYPE)
            .and_then(|value| value.to_str().ok())
            .unwrap_or_default()
            .to_owned();
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap()
            .to_vec();
        (status, content_type, body)
    }

    async fn send(app: &Router, request: Request) -> (StatusCode, String) {
        let (status, _, body) = send_full(app, request).await;
        (status, String::from_utf8_lossy(&body).into_owned())
    }

    fn get(uri: &str) -> Request {
        axum::http::Request::builder()
            .uri(uri)
            .body(Body::empty())
            .unwrap()
    }

    fn post(uri: &str, content_type: &str, body: impl Into<Body>) -> Request {
        axum::http::Request::builder()
            .method("POST")
            .uri(uri)
            .header(header::CONTENT_TYPE, content_type)
            .body(body.into())
            .unwrap()
    }

    #[tokio::test]
    async fn http_api() {
        let app = andorra_app();

        // JSON body in, JSON out.
        let (status, body) = send(&app, post("/route", "application/json", ROUTE_JSON)).await;
        assert_eq!(status, StatusCode::OK, "{body}");
        assert!(body.contains("\"trip\""), "{body}");

        // GET request carried in `?json=`.
        let query = form_urlencoded::Serializer::new(String::new())
            .append_pair("json", ROUTE_JSON)
            .finish();
        let (status, body) = send(&app, get(&format!("/route?{query}"))).await;
        assert_eq!(status, StatusCode::OK, "{body}");
        assert!(body.contains("\"trip\""), "{body}");

        // PBF in and out.
        let api = proto::Api {
            options: Some(proto::Options {
                format: proto::options::Format::Pbf as i32, // N.B.: Format::Json is the default
                costing_type: proto::costing::Type::Auto as i32,
                locations: vec![
                    proto::Location {
                        ll: valhalla::LatLon(42.50107, 1.51034).into(),
                        ..Default::default()
                    },
                    proto::Location {
                        ll: valhalla::LatLon(42.50627, 1.52173).into(),
                        ..Default::default()
                    },
                ],
                ..Default::default()
            }),
            ..Default::default()
        };
        let (status, content_type, body) = send_full(
            &app,
            post("/route", "application/x-protobuf", api.encode_to_vec()),
        )
        .await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(content_type, "application/x-protobuf");
        proto::Api::decode(body.as_slice()).expect("valid protobuf response");

        let (status, _) = send(&app, get("/status")).await;
        assert_eq!(status, StatusCode::OK);

        let (status, _) = send(&app, post("/route", "application/json", "not json{")).await;
        assert_eq!(status, StatusCode::BAD_REQUEST);

        // Valhalla treats every request error (like a single location here) as a 400.
        let one_location = r#"{"locations":[{"lat":42.50107,"lon":1.51034}],"costing":"auto"}"#;
        let (status, _) = send(&app, post("/route", "application/json", one_location)).await;
        assert_eq!(status, StatusCode::BAD_REQUEST);

        let (status, _) = send(&app, get("/unknown_path")).await;
        assert_eq!(status, StatusCode::NOT_FOUND);
    }
}
