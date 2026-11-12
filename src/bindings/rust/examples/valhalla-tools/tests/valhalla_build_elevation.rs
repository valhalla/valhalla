use std::collections::HashMap;
use std::fs;
use std::io::{self, Read, Write};
use std::net::TcpListener;
use std::num::NonZeroUsize;
use std::path::Path;
use std::process::{Command, Output};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

use flate2::write::GzEncoder;
use serde_json::json;
use tempfile::TempDir;
use tracing_subscriber::fmt::MakeWriter;
use valhalla_tools::elevation::{
    self, Compression, DownloadOptions, Options, Source, Summary, Tile,
};

/// Size of every SRTM 1 arc-second tile: 3601 x 3601 big-endian 16-bit samples.
const HGT_SIZE: usize = 25_934_402;

/// Stand-in elevation samples, small enough to keep the tests fast.
fn hgt(seed: u8) -> Vec<u8> {
    (0..5000_u32).map(|i| (i / 13) as u8 ^ seed).collect()
}

fn gzip(data: &[u8]) -> Vec<u8> {
    let mut encoder = GzEncoder::new(Vec::new(), flate2::Compression::default());
    encoder.write_all(data).unwrap();
    encoder.finish().unwrap()
}

fn http_response(status: &str, body: &[u8]) -> Vec<u8> {
    let mut response = format!(
        "HTTP/1.1 {status}\r\nContent-Type: application/octet-stream\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    )
    .into_bytes();
    response.extend_from_slice(body);
    response
}

fn ok(body: &[u8]) -> Vec<u8> {
    http_response("200 OK", body)
}

fn not_found() -> Vec<u8> {
    http_response("404 Not Found", b"")
}

/// Sends half the body without a Content-Length, then closes the connection, which the
/// client cannot tell from a complete body.
fn cut_without_length(body: &[u8]) -> Vec<u8> {
    let mut response =
        b"HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nConnection: close\r\n\r\n"
            .to_vec();
    response.extend_from_slice(&body[..body.len() / 2]);
    response
}

/// Announces more bytes than it sends, then closes the connection.
fn truncated(body: &[u8]) -> Vec<u8> {
    let mut response = http_response("200 OK", body);
    response.truncate(response.len() - body.len() / 2);
    response
}

/// Answers the `n`th request (from 0) for a path; an empty answer closes the connection.
type Responder = dyn Fn(&str, usize) -> Vec<u8> + Send + Sync;

/// A local HTTP server answering each connection on its own thread until the test ends.
struct FakeServer {
    url: String,
    requests: Arc<Mutex<Vec<String>>>,
    /// The most requests answered at the same time.
    max_in_flight: Arc<AtomicUsize>,
}

impl FakeServer {
    fn start(responder: impl Fn(&str, usize) -> Vec<u8> + Send + Sync + 'static) -> Self {
        let responder: Arc<Responder> = Arc::new(responder);
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/skadi", listener.local_addr().unwrap());
        let requests = Arc::new(Mutex::new(Vec::new()));
        let max_in_flight = Arc::new(AtomicUsize::new(0));
        let recorded = Arc::clone(&requests);
        let max = Arc::clone(&max_in_flight);
        thread::spawn(move || {
            let counts: Arc<Mutex<HashMap<String, usize>>> = Arc::default();
            let in_flight = Arc::new(AtomicUsize::new(0));
            for stream in listener.incoming() {
                let Ok(mut stream) = stream else { continue };
                let (responder, recorded, counts) = (
                    Arc::clone(&responder),
                    Arc::clone(&recorded),
                    Arc::clone(&counts),
                );
                let (in_flight, max) = (Arc::clone(&in_flight), Arc::clone(&max));
                thread::spawn(move || {
                    let Some(path) = read_request_path(&mut stream) else {
                        return;
                    };
                    let now = in_flight.fetch_add(1, Ordering::SeqCst) + 1;
                    max.fetch_max(now, Ordering::SeqCst);
                    recorded.lock().unwrap().push(path.clone());
                    let count = {
                        let mut counts = counts.lock().unwrap();
                        let count = counts.entry(path.clone()).or_default();
                        *count += 1;
                        *count - 1
                    };
                    let response = responder(&path, count);
                    let _ = stream.write_all(&response);
                    in_flight.fetch_sub(1, Ordering::SeqCst);
                });
            }
        });
        Self {
            url,
            requests,
            max_in_flight,
        }
    }

    /// A server nothing listens on.
    fn unreachable() -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/skadi", listener.local_addr().unwrap());
        drop(listener);
        Self {
            url,
            requests: Arc::default(),
            max_in_flight: Arc::default(),
        }
    }

    /// Serves each tile's gzipped bytes, answering 404 for any other path.
    fn serving(tiles: &[(&str, Vec<u8>)]) -> Self {
        let bodies: HashMap<String, Vec<u8>> = tiles
            .iter()
            .map(|(path, data)| (format!("/skadi/{path}.gz"), gzip(data)))
            .collect();
        Self::start(move |path, _| bodies.get(path).map_or_else(not_found, |body| ok(body)))
    }

    fn requests(&self) -> Vec<String> {
        let mut requests = self.requests.lock().unwrap().clone();
        requests.sort();
        requests
    }
}

fn read_request_path(stream: &mut impl Read) -> Option<String> {
    let mut request = Vec::new();
    let mut buffer = [0; 1024];
    while !request.ends_with(b"\r\n\r\n") {
        let read = stream.read(&mut buffer).ok()?;
        if read == 0 {
            return None;
        }
        request.extend_from_slice(&buffer[..read]);
    }
    let line = String::from_utf8_lossy(&request);
    line.split(' ').nth(1).map(str::to_owned)
}

fn build_elevation(server_url: &str) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_valhalla_build_elevation"));
    command.env("VALHALLA_ELEVATION_TILES_URL", server_url);
    command
}

fn assert_success(output: &Output) {
    assert!(
        output.status.success(),
        "exit {:?}: {}",
        output.status.code(),
        String::from_utf8_lossy(&output.stderr)
    );
}

fn stderr(output: &Output) -> String {
    String::from_utf8_lossy(&output.stderr).into_owned()
}

/// The messages of the log lines, without the Python format's timestamp.
fn messages(output: &Output) -> Vec<String> {
    stderr(output)
        .lines()
        .filter_map(|line| line.split_once(' ').map(|(_, rest)| rest))
        .filter_map(|rest| {
            rest.split_once(' ')
                .map(|(_, rest)| rest.trim_start().to_owned())
        })
        .collect()
}

/// Files below `dir`, relative and sorted.
fn files(dir: &Path) -> Vec<String> {
    let mut found = Vec::new();
    let mut pending = vec![dir.to_path_buf()];
    while let Some(current) = pending.pop() {
        let Ok(entries) = fs::read_dir(&current) else {
            continue;
        };
        for entry in entries {
            let path = entry.unwrap().path();
            if path.is_dir() {
                pending.push(path);
            } else {
                let relative = path.strip_prefix(dir).unwrap();
                found.push(relative.to_string_lossy().into_owned());
            }
        }
    }
    found.sort();
    found
}

fn assert_critical(output: &Output, message: &str) {
    let expected = format!("CRITICAL: {message}");
    assert!(
        output.status.code() == Some(1) && messages(output) == [expected.clone()],
        "exit {:?}, expected {expected:?}, stderr: {}",
        output.status.code(),
        stderr(output)
    );
}

fn assert_usage_error(output: &Output) {
    assert_eq!(output.status.code(), Some(2), "stderr: {}", stderr(output));
}

// Binary: success paths.

#[test]
fn by_default_the_gzipped_tile_is_stored_as_served() {
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(1))]);

    let output = build_elevation(&server.url)
        .args(["-b", "5.1,52.1,5.2,52.2", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    let stored = fs::read(dir.path().join("N52/N52E005.hgt.gz")).unwrap();
    assert_eq!(stored, gzip(&hgt(1)));
}

#[test]
fn decompress_stores_the_raw_hgt() {
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(2))]);

    let output = build_elevation(&server.url)
        .args(["-d", "-b", "5.1,52.1,5.2,52.2", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(
        fs::read(dir.path().join("N52/N52E005.hgt")).unwrap(),
        hgt(2)
    );
}

fn decode_lz4(frame: &[u8]) -> Vec<u8> {
    let mut decoded = Vec::new();
    lz4::Decoder::new(frame)
        .unwrap()
        .read_to_end(&mut decoded)
        .unwrap();
    decoded
}

#[test]
fn lz4_stores_a_frame_of_the_raw_hgt() {
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::serving(&[("S01/S01W001.hgt", hgt(3))]);

    let output = build_elevation(&server.url)
        .args(["-z", "-b", "-0.5,-0.5,-0.4,-0.4", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    let frame = fs::read(dir.path().join("S01/S01W001.hgt.lz4")).unwrap();
    assert_eq!(decode_lz4(&frame), hgt(3));
}

#[test]
fn a_full_size_lz4_tile_declares_the_hgt_size_in_its_frame_header() {
    let dir = tempfile::tempdir().unwrap();
    let data: Vec<u8> = (0..HGT_SIZE).map(|i| (i / 7202 % 251) as u8).collect();
    let server = FakeServer::serving(&[("N00/N00E000.hgt", data.clone())]);

    let output = build_elevation(&server.url)
        .args(["--lz4", "-b", "0.5,0.5,0.6,0.6", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    let frame = fs::read(dir.path().join("N00/N00E000.hgt.lz4")).unwrap();
    // Magic; version 1, linked blocks, content size, no checksums; 4 MB blocks; content size.
    let mut header = vec![0x04, 0x22, 0x4d, 0x18, 0x48, 0x70];
    header.extend_from_slice(&(HGT_SIZE as u64).to_le_bytes());
    assert_eq!(
        (&frame[..14], decode_lz4(&frame) == data),
        (&header[..], true)
    );
}

#[test]
fn an_existing_tile_is_skipped_without_a_request() {
    let dir = tempfile::tempdir().unwrap();
    fs::create_dir(dir.path().join("N52")).unwrap();
    fs::write(dir.path().join("N52/N52E005.hgt.gz"), b"kept").unwrap();
    let server = FakeServer::serving(&[]);

    let output = build_elevation(&server.url)
        .args(["-v", "-b", "5.1,52.1,5.2,52.2", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(
        (server.requests(), messages(&output)),
        (
            Vec::new(),
            vec!["INFO: Downloaded 0 tiles. Exiting.".to_owned()]
        )
    );
}

#[test]
fn only_a_tile_in_the_requested_format_counts_as_existing() {
    let dir = tempfile::tempdir().unwrap();
    fs::create_dir(dir.path().join("N52")).unwrap();
    fs::write(dir.path().join("N52/N52E005.hgt.gz"), gzip(&hgt(4))).unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(4))]);

    let output = build_elevation(&server.url)
        .args(["-d", "-b", "5.1,52.1,5.2,52.2", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(server.requests(), ["/skadi/N52/N52E005.hgt.gz"]);
}

const NEGATIVE_BBOX_TILES: [&str; 4] = [
    "/skadi/N47/N47E000.hgt.gz",
    "/skadi/N47/N47W001.hgt.gz",
    "/skadi/N48/N48E000.hgt.gz",
    "/skadi/N48/N48W001.hgt.gz",
];

fn serve_negative_bbox_tiles() -> FakeServer {
    FakeServer::serving(&[
        ("N47/N47E000.hgt", hgt(5)),
        ("N47/N47W001.hgt", hgt(6)),
        ("N48/N48E000.hgt", hgt(7)),
        ("N48/N48W001.hgt", hgt(8)),
    ])
}

#[test]
fn a_bbox_starting_with_a_minus_is_accepted_as_a_separate_argument() {
    let dir = tempfile::tempdir().unwrap();
    let server = serve_negative_bbox_tiles();

    let output = build_elevation(&server.url)
        .args(["-b", "-0.45,47.56,0.92,48.49", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(server.requests(), NEGATIVE_BBOX_TILES);
}

#[test]
fn a_bbox_starting_with_a_minus_is_accepted_after_an_equals_sign() {
    let dir = tempfile::tempdir().unwrap();
    let server = serve_negative_bbox_tiles();

    let output = build_elevation(&server.url)
        .args(["-b=-0.45,47.56,0.92,48.49", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(server.requests(), NEGATIVE_BBOX_TILES);
}

#[test]
fn every_tile_downloaded_is_counted_once() {
    let dir = tempfile::tempdir().unwrap();
    let server = serve_negative_bbox_tiles();

    let output = build_elevation(&server.url)
        .args([
            "-v",
            "-p",
            "2",
            "--from-bbox",
            "-0.45,47.56,0.92,48.49",
            "-o",
        ])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    let last = messages(&output).last().cloned();
    assert_eq!(last.as_deref(), Some("INFO: Downloaded 4 tiles. Exiting."));
}

fn write_l_shaped_geojson(dir: &Path) {
    //  __
    // | |__
    // |___|
    let collection = json!({"type": "FeatureCollection", "features": [{"type": "Feature", "geometry": {
        "type": "Polygon",
        "coordinates": [[[0, 0], [0, 3], [1, 3], [1, 1], [3, 1], [3, 0], [0, 0]]]
    }}]});
    fs::create_dir_all(dir).unwrap();
    fs::write(dir.join("l.geojson"), collection.to_string()).unwrap();
}

#[test]
fn a_geojson_dir_downloads_the_tiles_its_polygons_touch() {
    let dir = tempfile::tempdir().unwrap();
    write_l_shaped_geojson(&dir.path().join("geojson"));
    let server = FakeServer::start(|_, _| ok(&gzip(&hgt(9))));

    let output = build_elevation(&server.url)
        .arg("-g")
        .arg(dir.path().join("geojson"))
        .arg("-o")
        .arg(dir.path().join("out"))
        .output()
        .unwrap();
    assert_success(&output);

    let expected = [
        "N00/N00E000.hgt.gz",
        "N00/N00E001.hgt.gz",
        "N00/N00E002.hgt.gz",
        "N01/N01E000.hgt.gz",
        "N01/N01E001.hgt.gz",
        "N01/N01E002.hgt.gz",
        "N02/N02E000.hgt.gz",
        "N02/N02E001.hgt.gz",
    ];
    assert_eq!(files(&dir.path().join("out")), expected);
}

/// A tile tree like test/data/utrecht_tiles, whose only level 2 tile lies in N52E005.
fn utrecht_graph(dir: &Path) {
    for name in ["0/003/196.gph", "1/051/305.gph", "2/000/818/660.gph"] {
        let path = dir.join(name);
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(path, b"").unwrap();
    }
}

#[test]
fn from_tiles_downloads_the_tiles_under_the_level_2_graph_tiles() {
    let dir = tempfile::tempdir().unwrap();
    utrecht_graph(&dir.path().join("tiles"));
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(10))]);
    let config = json!({
        "mjolnir": {"tile_dir": dir.path().join("tiles")},
        "additional_data": {"elevation": dir.path().join("elevation")},
    });

    let output = build_elevation(&server.url)
        .args(["-t", "-i", &config.to_string()])
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(files(&dir.path().join("elevation")), ["N52/N52E005.hgt.gz"]);
}

#[test]
fn the_config_file_gives_the_output_directory() {
    let dir = tempfile::tempdir().unwrap();
    let config = json!({"additional_data": {"elevation": dir.path().join("from_config")}});
    let config_path = dir.path().join("valhalla.json");
    fs::write(&config_path, config.to_string()).unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(11))]);

    let output = build_elevation(&server.url)
        .args(["-b", "5.1,52.1,5.2,52.2", "-c"])
        .arg(&config_path)
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(
        files(&dir.path().join("from_config")),
        ["N52/N52E005.hgt.gz"]
    );
}

#[test]
fn an_empty_configured_output_directory_falls_back_to_elevation() {
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(12))]);

    let output = build_elevation(&server.url)
        .current_dir(dir.path())
        .args(["-b", "5.1,52.1,5.2,52.2", "-i"])
        .arg(json!({"additional_data": {"elevation": ""}}).to_string())
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(files(&dir.path().join("elevation")), ["N52/N52E005.hgt.gz"]);
}

#[test]
fn outdir_overrides_the_config() {
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(13))]);
    let config = json!({"additional_data": {"elevation": dir.path().join("from_config")}});

    let output = build_elevation(&server.url)
        .args(["-b", "5.1,52.1,5.2,52.2", "-i", &config.to_string(), "-o"])
        .arg(dir.path().join("outdir"))
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(
        (
            files(&dir.path().join("outdir")),
            dir.path().join("from_config").exists()
        ),
        (vec!["N52/N52E005.hgt.gz".to_owned()], false)
    );
}

#[test]
fn without_v_a_successful_run_logs_nothing() {
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(14))]);

    let output = build_elevation(&server.url)
        .args(["-b", "5.1,52.1,5.2,52.2", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(stderr(&output), "");
}

#[test]
fn vv_logs_the_bbox_and_the_tiles_as_python_reprs() {
    let dir = tempfile::tempdir().unwrap();
    let server = serve_negative_bbox_tiles();

    let output = build_elevation(&server.url)
        .args(["-vv", "-p", "1", "-b", "-0.45,47.56,0.92,48"])
        .arg("-o")
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    let messages = messages(&output);
    let expected_start = [
        "DEBUG: Received valid bbox: [-0.45, 47.56, 0.92, 48.0]",
        "DEBUG: [Tile(name='N47E000.hgt', dir='N47'), Tile(name='N47W001.hgt', dir='N47')]",
        "INFO: Downloading tile N47E000.hgt",
        "DEBUG: Downloading tile N47/N47E000.hgt for the 0th time.",
        "DEBUG: Successfully downloaded tile N47/N47E000.hgt",
    ];
    assert_eq!(messages[..5], expected_start);
}

#[test]
fn the_valhalla_log_format_has_bracketed_levels() {
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(15))]);
    let config = json!({"logging": {"type": "std_err", "color": false}});

    let output = build_elevation(&server.url)
        .args(["-v", "--log-format", "valhalla", "-b", "5.1,52.1,5.2,52.2"])
        .args(["-i", &config.to_string(), "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    let stderr = stderr(&output);
    assert!(
        stderr
            .lines()
            .last()
            .unwrap()
            .ends_with(" [INFO] Downloaded 1 tiles. Exiting."),
        "stderr: {stderr}"
    );
}

#[cfg(unix)]
#[test]
fn a_tile_gets_the_permissions_of_a_newly_created_file() {
    use std::os::unix::fs::PermissionsExt;

    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(16))]);

    let output = build_elevation(&server.url)
        .args(["-b", "5.1,52.1,5.2,52.2", "-o"])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    let reference = dir.path().join("reference");
    fs::write(&reference, b"").unwrap();
    let mode = |path: &Path| fs::metadata(path).unwrap().permissions().mode() & 0o777;
    assert_eq!(
        mode(&dir.path().join("N52/N52E005.hgt.gz")),
        mode(&reference)
    );
}

// Binary: errors.

fn run_without_server(args: &[&str]) -> Output {
    build_elevation("http://127.0.0.1:9/skadi")
        .args(args)
        .output()
        .unwrap()
}

#[test]
fn without_outdir_or_config_the_run_fails() {
    let output = run_without_server(&["-b", "5,52,6,53"]);
    assert_critical(&output, "Either config or outdir is required.");
}

#[test]
fn an_empty_config_counts_as_no_config() {
    let dir = tempfile::tempdir().unwrap();
    let config_path = dir.path().join("valhalla.json");
    fs::write(&config_path, "{}").unwrap();
    let output = run_without_server(&["-b", "5,52,6,53", "-c", config_path.to_str().unwrap()]);
    assert_critical(&output, "Either config or outdir is required.");
}

#[test]
fn without_a_method_the_run_fails() {
    let output = run_without_server(&["-o", "out"]);
    assert_critical(&output, "No download method specified.");
}

#[test]
fn an_empty_bbox_counts_as_no_method() {
    let output = run_without_server(&["-o", "out", "-b", ""]);
    assert_critical(&output, "No download method specified.");
}

#[test]
fn from_tiles_without_a_config_fails() {
    let output = run_without_server(&["-t", "-o", "out"]);
    assert_critical(&output, "--from-tiles requires a config to be specified.");
}

#[test]
fn a_config_without_an_elevation_dir_fails() {
    let output = run_without_server(&["-t", "-i", r#"{"mjolnir": {"tile_dir": "tiles"}}"#]);
    assert_critical(
        &output,
        "the configuration has no 'additional_data.elevation'",
    );
}

#[test]
fn from_tiles_without_tile_dir_fails() {
    let output = run_without_server(&["-t", "-o", "out", "-i", r#"{"mjolnir": {}}"#]);
    assert_critical(&output, "the configuration has no 'mjolnir.tile_dir'");
}

#[test]
fn from_tiles_with_a_missing_graph_dir_fails() {
    let dir = tempfile::tempdir().unwrap();
    let missing = dir.path().join("tiles");
    let config = json!({"mjolnir": {"tile_dir": missing}});
    let output = run_without_server(&["-t", "-o", "out", "-i", &config.to_string()]);
    let message = format!("Graph directory {} does not exist.", missing.display());
    assert_critical(&output, &message);
}

#[test]
fn a_graph_tile_with_a_non_numeric_name_fails() {
    let dir = tempfile::tempdir().unwrap();
    let bad = dir.path().join("tiles/2/000/abc.gph");
    fs::create_dir_all(bad.parent().unwrap()).unwrap();
    fs::write(&bad, b"").unwrap();
    let config = json!({"mjolnir": {"tile_dir": dir.path().join("tiles")}});
    let output = run_without_server(&["-t", "-o", "out", "-i", &config.to_string()]);
    assert_critical(
        &output,
        &format!("Couldn't get a tile ID from {}", bad.display()),
    );
}

#[test]
fn a_bbox_with_three_values_fails() {
    let output = run_without_server(&["-o", "out", "-b", "5,52,6"]);
    assert_critical(
        &output,
        "BBOX 5,52,6 is not a comma-separated string of coordinates.",
    );
}

#[test]
fn a_reversed_bbox_fails_with_python_reprs() {
    let output = run_without_server(&["-o", "out", "-b", "6,52,5,53"]);
    assert_critical(&output, "Bbox invalid: [6.0, 52.0, 5.0, 53.0]");
}

#[test]
fn a_bbox_with_nan_fails() {
    let output = run_without_server(&["-o", "out", "-b", "5,nan,6,53"]);
    assert_critical(&output, "Bbox invalid: [5.0, nan, 6.0, 53.0]");
}

#[test]
fn a_missing_geojson_dir_fails() {
    let dir = tempfile::tempdir().unwrap();
    let missing = dir.path().join("geojson");
    let output = run_without_server(&["-o", "out", "-g", missing.to_str().unwrap()]);
    let message = format!(
        "Geojson directory does not exist or contains no GeoJSON files: {}",
        missing.display()
    );
    assert_critical(&output, &message);
}

#[test]
fn a_geojson_dir_without_geojson_files_fails() {
    let dir = tempfile::tempdir().unwrap();
    fs::write(dir.path().join("notes.json"), "{}").unwrap();
    let output = run_without_server(&["-o", "out", "-g", dir.path().to_str().unwrap()]);
    let message = format!(
        "Geojson directory does not exist or contains no GeoJSON files: {}",
        fs::canonicalize(dir.path()).unwrap().display()
    );
    assert_critical(&output, &message);
}

#[test]
fn an_invalid_geojson_file_fails() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("broken.geojson");
    fs::write(&path, "{").unwrap();
    let output = run_without_server(&["-o", "out", "-g", dir.path().to_str().unwrap()]);
    let messages = messages(&output);
    let expected = format!("CRITICAL: invalid JSON in {}: ", path.display());
    assert!(
        output.status.code() == Some(1)
            && messages.len() == 1
            && messages[0].starts_with(&expected),
        "stderr: {}",
        stderr(&output)
    );
}

#[test]
fn a_geojson_feature_with_malformed_coordinates_fails() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("bad.geojson");
    let collection =
        json!({"features": [{"geometry": {"type": "Polygon", "coordinates": [[["a", 0]]]}}]});
    fs::write(&path, collection.to_string()).unwrap();
    let output = run_without_server(&["-o", "out", "-g", dir.path().to_str().unwrap()]);
    let message = format!(
        "{}: expected a FeatureCollection with valid Polygon or MultiPolygon geometries",
        path.display()
    );
    assert_critical(&output, &message);
}

#[test]
fn an_invalid_config_file_fails() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("valhalla.json");
    fs::write(&path, "{").unwrap();
    let output =
        run_without_server(&["-o", "out", "-b", "5,52,6,53", "-c", path.to_str().unwrap()]);
    assert_eq!(output.status.code(), Some(1));
}

/// A file where the tile directory should be makes the tile fail without any retry delay.
fn run_with_unwritable_tile(verbosity: &[&str]) -> Output {
    let dir = tempfile::tempdir().unwrap();
    fs::write(dir.path().join("N52"), b"").unwrap();
    let server = FakeServer::serving(&[("N53/N53E005.hgt", hgt(17))]);
    build_elevation(&server.url)
        .args(verbosity)
        .args(["-b", "5.1,52.1,5.2,53.2", "-o"])
        .arg(dir.path())
        .output()
        .unwrap()
}

#[test]
fn a_failed_tile_makes_the_run_fail_after_the_others() {
    let output = run_with_unwritable_tile(&[]);
    let last = messages(&output).last().cloned();
    assert_eq!(
        (output.status.code(), last.as_deref()),
        (
            Some(1),
            Some("CRITICAL: 1 of 2 elevation tiles could not be downloaded.")
        )
    );
}

#[test]
fn a_local_error_is_shown_as_critical_without_v() {
    let output = run_with_unwritable_tile(&[]);
    let messages = messages(&output);
    assert!(
        messages.len() == 2 && messages[0].starts_with("CRITICAL: Could not create directory "),
        "stderr: {}",
        stderr(&output)
    );
}

#[test]
fn a_repeated_option_keeps_its_last_value_like_argparse() {
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::serving(&[("N52/N52E005.hgt", hgt(18))]);

    let output = build_elevation(&server.url)
        .args([
            "-b",
            "1,2,3",
            "-b",
            "5.1,52.1,5.2,52.2",
            "-o",
            "ignored",
            "-o",
        ])
        .arg(dir.path())
        .output()
        .unwrap();
    assert_success(&output);

    assert_eq!(files(dir.path()), ["N52/N52E005.hgt.gz"]);
}

#[test]
fn the_long_help_mentions_the_tiles_url_variable() {
    let output = build_elevation("").arg("--help").output().unwrap();
    let help = String::from_utf8_lossy(&output.stdout);
    assert!(help.contains("VALHALLA_ELEVATION_TILES_URL"), "{help}");
}

#[test]
fn a_zero_parallelism_is_a_usage_error() {
    assert_usage_error(&run_without_server(&[
        "-p",
        "0",
        "-o",
        "out",
        "-b",
        "5,52,6,53",
    ]));
}

#[test]
fn a_negative_parallelism_is_a_usage_error() {
    assert_usage_error(&run_without_server(&[
        "-p",
        "-1",
        "-o",
        "out",
        "-b",
        "5,52,6,53",
    ]));
}

#[test]
fn two_methods_are_a_usage_error() {
    assert_usage_error(&run_without_server(&["-t", "-b", "5,52,6,53", "-o", "out"]));
}

#[test]
fn a_geojson_dir_and_a_bbox_are_a_usage_error() {
    assert_usage_error(&run_without_server(&[
        "-g",
        "dir",
        "-b",
        "5,52,6,53",
        "-o",
        "out",
    ]));
}

#[test]
fn decompress_and_lz4_together_are_a_usage_error() {
    assert_usage_error(&run_without_server(&[
        "-d",
        "-z",
        "-b",
        "5,52,6,53",
        "-o",
        "out",
    ]));
}

#[test]
fn long_options_can_be_abbreviated_like_argparse() {
    let output = run_without_server(&["--from-b", "6,52,5,53", "--out", "out"]);
    assert_critical(&output, "Bbox invalid: [6.0, 52.0, 5.0, 53.0]");
}

// Library: retries, against the fake server with no delay between attempts.

/// Log lines of every download in this test binary, with their tile span.
static LOG: Mutex<Vec<u8>> = Mutex::new(Vec::new());

struct Capture;

impl Write for Capture {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        LOG.lock().unwrap().extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl<'a> MakeWriter<'a> for Capture {
    type Writer = Capture;

    fn make_writer(&'a self) -> Self::Writer {
        Capture
    }
}

/// One global subscriber: a thread-local one would miss events of the download threads.
fn capture_logs() {
    static INSTALLED: OnceLock<()> = OnceLock::new();
    INSTALLED.get_or_init(|| {
        let subscriber = tracing_subscriber::fmt()
            .with_writer(Capture)
            .with_max_level(tracing::Level::DEBUG)
            .without_time()
            .with_target(false)
            .finish();
        tracing::subscriber::set_global_default(subscriber).unwrap();
    });
}

/// Messages logged for `tile`, which each test picks unique.
fn tile_messages(tile: &Tile) -> Vec<String> {
    let log = String::from_utf8_lossy(&LOG.lock().unwrap()).into_owned();
    let span = format!("elevation_tile{{tile={}}}: ", tile.name);
    let is_event = |line: &str| {
        let line = line.trim_start();
        ["ERROR ", "WARN ", "INFO ", "DEBUG ", "TRACE "]
            .iter()
            .any(|level| line.starts_with(level))
    };
    // Each event is written at once, so the lines of a multi-line message follow its first.
    let mut messages: Vec<String> = Vec::new();
    let mut in_tile = false;
    for line in log.lines() {
        if is_event(line) {
            in_tile = false;
            if let Some((_, message)) = line.split_once(&span) {
                messages.push(message.to_owned());
                in_tile = true;
            }
        } else if in_tile && let Some(last) = messages.last_mut() {
            last.push('\n');
            last.push_str(line);
        }
    }
    messages
}

struct Download {
    dir: TempDir,
    tile: Tile,
}

impl Download {
    /// Tiles at distinct longitudes keep the tests' log lines apart.
    fn new(x: i64) -> Self {
        capture_logs();
        Self {
            dir: tempfile::tempdir().unwrap(),
            tile: Tile::at(x, 10),
        }
    }

    fn path(&self) -> String {
        format!("/skadi/{}/{}.gz", self.tile.dir, self.tile.name)
    }

    fn run(&self, server: &FakeServer, compression: Compression, delays: &[Duration]) -> Summary {
        elevation::download(
            std::slice::from_ref(&self.tile),
            &DownloadOptions {
                outdir: self.dir.path(),
                compression,
                parallelism: NonZeroUsize::MIN,
                base_url: &server.url,
                retry_delays: delays,
            },
        )
    }

    fn files(&self) -> Vec<String> {
        files(self.dir.path())
    }

    fn messages(&self) -> Vec<String> {
        tile_messages(&self.tile)
    }
}

const NO_DELAYS: [Duration; 5] = [Duration::ZERO; 5];

const DOWNLOADED: Summary = Summary {
    downloaded: 1,
    failed: 0,
};

const FAILED: Summary = Summary {
    downloaded: 0,
    failed: 1,
};

#[test]
fn an_http_error_is_retried_until_the_download_succeeds() {
    let download = Download::new(101);
    let server = FakeServer::start(|_, attempt| match attempt {
        0 => http_response("503 Service Unavailable", b""),
        _ => ok(&gzip(&hgt(20))),
    });

    let summary = download.run(&server, Compression::Gzip, &NO_DELAYS);

    assert_eq!(summary, DOWNLOADED);
    let expected = [
        "Downloading tile N10E101.hgt",
        "Downloading tile N10/N10E101.hgt for the 0th time.",
        "Download failed with HTTP error 503: Service Unavailable.\nTrying again...",
        "Downloading tile N10/N10E101.hgt for the 1th time.",
        "Successfully downloaded tile N10/N10E101.hgt",
    ];
    assert_eq!(download.messages(), expected);
}

#[test]
fn a_tile_failing_every_attempt_is_counted_as_failed() {
    let download = Download::new(102);
    let server = FakeServer::start(|_, _| not_found());

    let summary = download.run(&server, Compression::Gzip, &NO_DELAYS);

    assert_eq!(
        (summary, server.requests().len(), download.files()),
        (FAILED, 5, Vec::new())
    );
}

#[test]
fn a_tile_failing_every_attempt_is_reported_as_not_downloaded() {
    let download = Download::new(103);
    let server = FakeServer::start(|_, _| not_found());

    download.run(&server, Compression::Gzip, &NO_DELAYS);

    let messages = download.messages();
    assert_eq!(
        messages.last().map(String::as_str),
        Some("Tile N10/N10E103.hgt couldn't be downloaded...")
    );
}

#[test]
fn there_is_one_attempt_per_retry_delay() {
    let download = Download::new(104);
    let server = FakeServer::start(|_, _| not_found());

    download.run(&server, Compression::Gzip, &[Duration::ZERO; 2]);

    assert_eq!(server.requests().len(), 2);
}

#[test]
fn each_retry_delay_is_waited_before_its_attempt() {
    let download = Download::new(105);
    let server = FakeServer::start(|_, _| not_found());
    let delays = [
        Duration::ZERO,
        Duration::from_millis(150),
        Duration::from_millis(150),
    ];

    let start = Instant::now();
    download.run(&server, Compression::Gzip, &delays);

    assert!(start.elapsed() >= Duration::from_millis(300));
}

#[test]
fn a_closed_connection_is_retried_as_a_transport_error() {
    let download = Download::new(106);
    let server = FakeServer::start(|_, attempt| match attempt {
        0 => Vec::new(),
        _ => ok(&gzip(&hgt(21))),
    });

    let summary = download.run(&server, Compression::Gzip, &NO_DELAYS);

    let messages = download.messages();
    let failure = "Download failed of elevation tile N10/N10E106.hgt: ";
    assert!(
        summary == DOWNLOADED
            && messages[2].starts_with(failure)
            && messages[2].ends_with(".\nTrying again.."),
        "{summary:?} {messages:?}"
    );
}

#[test]
fn a_body_cut_short_is_retried_and_leaves_no_partial_tile() {
    let download = Download::new(107);
    let body = gzip(&hgt(22));
    let server = FakeServer::start(move |_, attempt| match attempt {
        0 => truncated(&body),
        _ => ok(&body),
    });

    let summary = download.run(&server, Compression::Gzip, &NO_DELAYS);

    assert_eq!(
        (summary, download.files()),
        (DOWNLOADED, vec!["N10/N10E107.hgt.gz".to_owned()])
    );
}

#[test]
fn a_body_cut_short_on_every_attempt_leaves_nothing_behind() {
    let download = Download::new(108);
    let body = gzip(&hgt(23));
    let server = FakeServer::start(move |_, _| truncated(&body));

    let summary = download.run(&server, Compression::Gzip, &NO_DELAYS);

    assert_eq!((summary, download.files()), (FAILED, Vec::new()));
}

#[test]
fn a_corrupted_gzip_is_retried_when_decompressing() {
    let download = Download::new(109);
    let server = FakeServer::start(|_, attempt| match attempt {
        0 => ok(b"not gzip"),
        _ => ok(&gzip(&hgt(24))),
    });

    let summary = download.run(&server, Compression::Uncompressed, &NO_DELAYS);

    let stored = fs::read(download.dir.path().join("N10/N10E109.hgt")).ok();
    let messages = download.messages();
    let corrupted = "Decompression error on tile N10/N10E109.hgt: ";
    assert!(
        summary == DOWNLOADED
            && stored == Some(hgt(24))
            && messages[2].starts_with(corrupted)
            && messages[2].ends_with(". Likely a corrupted download."),
        "{summary:?} {messages:?}"
    );
}

#[test]
fn a_tile_corrupted_on_every_attempt_is_reported_and_leaves_nothing_behind() {
    let download = Download::new(110);
    let server = FakeServer::start(|_, _| ok(b"not gzip"));

    let summary = download.run(&server, Compression::Lz4, &NO_DELAYS);

    let messages = download.messages();
    assert_eq!(
        (summary, download.files(), messages.last().cloned()),
        (
            FAILED,
            Vec::new(),
            Some("Tile N10/N10E110.hgt was corrupted, removing it...".to_owned())
        )
    );
}

#[test]
fn a_tile_corrupted_once_then_unavailable_is_reported_as_corrupted() {
    let download = Download::new(111);
    let server = FakeServer::start(|_, attempt| match attempt {
        0 => ok(b"not gzip"),
        _ => not_found(),
    });

    download.run(&server, Compression::Uncompressed, &NO_DELAYS);

    assert_eq!(
        download.messages().last().map(String::as_str),
        Some("Tile N10/N10E111.hgt was corrupted, removing it...")
    );
}

#[test]
fn a_body_that_is_not_gzip_is_retried_as_corrupted_in_the_gzip_mode() {
    let download = Download::new(112);
    let server = FakeServer::start(|_, attempt| match attempt {
        0 => http_response("200 OK", b"<html>proxy error</html>"),
        _ => ok(&gzip(&hgt(28))),
    });

    let summary = download.run(&server, Compression::Gzip, &NO_DELAYS);

    let stored = fs::read(download.dir.path().join("N10/N10E112.hgt.gz")).ok();
    let messages = download.messages();
    assert!(
        summary == DOWNLOADED
            && stored == Some(gzip(&hgt(28)))
            && messages[2].starts_with("Decompression error on tile N10/N10E112.hgt: "),
        "{summary:?} {messages:?}"
    );
}

#[test]
fn a_body_cut_short_without_a_content_length_is_never_stored() {
    let download = Download::new(115);
    let body = gzip(&hgt(29));
    let server = FakeServer::start(move |_, _| cut_without_length(&body));

    let summary = download.run(&server, Compression::Gzip, &NO_DELAYS);

    let messages = download.messages();
    assert_eq!(
        (summary, download.files(), messages.last().cloned()),
        (
            FAILED,
            Vec::new(),
            Some("Tile N10/N10E115.hgt was corrupted, removing it...".to_owned())
        )
    );
}

#[test]
fn a_gzip_content_encoding_is_not_decoded() {
    let download = Download::new(116);
    let body = gzip(&hgt(30));
    let served = body.clone();
    let server = FakeServer::start(move |_, _| {
        let mut response = format!(
            "HTTP/1.1 200 OK\r\nContent-Encoding: gzip\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
            served.len()
        )
        .into_bytes();
        response.extend_from_slice(&served);
        response
    });

    download.run(&server, Compression::Gzip, &NO_DELAYS);

    let stored = fs::read(download.dir.path().join("N10/N10E116.hgt.gz")).ok();
    assert_eq!(stored, Some(body));
}

#[test]
fn stale_temporary_files_of_the_tile_are_removed() {
    let download = Download::new(117);
    let dir = download.dir.path().join("N10");
    fs::create_dir_all(&dir).unwrap();
    fs::write(dir.join(".N10E117.hgt.gz.Ab12Cd.part"), b"partial").unwrap();
    fs::write(dir.join(".N10E118.hgt.gz.Ab12Cd.part"), b"other tile").unwrap();
    let server = FakeServer::serving(&[("N10/N10E117.hgt", hgt(31))]);

    download.run(&server, Compression::Gzip, &NO_DELAYS);

    assert_eq!(
        download.files(),
        ["N10/.N10E118.hgt.gz.Ab12Cd.part", "N10/N10E117.hgt.gz"]
    );
}

#[test]
fn an_unreachable_server_fails_every_attempt() {
    let download = Download::new(113);
    let server = FakeServer::unreachable();

    let summary = download.run(&server, Compression::Gzip, &NO_DELAYS);

    assert_eq!(summary, FAILED);
}

#[test]
fn tiles_are_shared_between_the_download_threads() {
    capture_logs();
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::start(|_, _| {
        thread::sleep(Duration::from_millis(200));
        ok(&gzip(&hgt(25)))
    });
    let tiles: Vec<_> = (0..6).map(|x| Tile::at(120 + x, 10)).collect();

    let summary = elevation::download(
        &tiles,
        &DownloadOptions {
            outdir: dir.path(),
            compression: Compression::Gzip,
            parallelism: NonZeroUsize::new(3).unwrap(),
            base_url: &server.url,
            retry_delays: &NO_DELAYS,
        },
    );

    let overlapped = server.max_in_flight.load(Ordering::SeqCst) >= 2;
    assert_eq!((summary.downloaded, overlapped), (6, true));
}

#[test]
fn a_run_with_a_failed_tile_returns_an_error_after_downloading_the_others() {
    capture_logs();
    let dir = tempfile::tempdir().unwrap();
    let server = FakeServer::start(|path, _| {
        if path.contains("E130") {
            not_found()
        } else {
            ok(&gzip(&hgt(26)))
        }
    });

    let result = elevation::run(&Options {
        config: None,
        inline_config: Some("{}".to_owned()),
        outdir: Some(dir.path().to_path_buf()),
        source: Some(Source::Bbox("130.5,10.5,131.5,10.6".to_owned())),
        compression: Compression::Gzip,
        parallelism: NonZeroUsize::MIN,
        base_url: server.url.clone(),
        retry_delays: &NO_DELAYS,
    });

    let message = result.err().map(|error| error.to_string());
    assert_eq!(
        (message.as_deref(), files(dir.path())),
        (
            Some("1 of 2 elevation tiles could not be downloaded."),
            vec!["N10/N10E131.hgt.gz".to_owned()]
        )
    );
}

#[test]
fn a_mirror_url_with_a_trailing_slash_is_accepted() {
    let download = Download::new(114);
    let server = FakeServer::serving(&[("N10/N10E114.hgt", hgt(27))]);
    let mirror = FakeServer {
        url: format!("{}/", server.url),
        ..FakeServer::unreachable()
    };

    let summary = download.run(&mirror, Compression::Gzip, &NO_DELAYS);

    assert_eq!(
        (summary, server.requests()),
        (DOWNLOADED, vec![download.path()])
    );
}
