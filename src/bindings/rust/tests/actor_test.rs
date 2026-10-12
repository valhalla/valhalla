#![cfg(feature = "proto")]

use valhalla::{
    Actor, ConfigBuilder, Error, LatLon, Response,
    proto::{self, options::Format},
};

const ANDORRA_TILES: &str = "tests/andorra/tiles.tar";
const ANDORRA_TRAFFIC: &str = "tests/andorra/traffic.tar";

const ANDORRA_TEST_LOC_1: LatLon = LatLon(42.50107335756198, 1.510341967860551); // Sant Julia de Loria
const ANDORRA_TEST_LOC_2: LatLon = LatLon(42.50627089323736, 1.521734167223563); // Andorra la Vella

#[test]
fn smoke() {
    let config = ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: ANDORRA_TILES.into(),
            traffic_extract: ANDORRA_TRAFFIC.into(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    let actor = Actor::new(&config);
    assert!(actor.is_ok());
}

#[test]
fn request_response_format() {
    type CheckFn = fn(&Result<Response, Error>) -> Result<(), String>;
    fn expect_json(response: &Result<Response, Error>) -> Result<(), String> {
        match response {
            Ok(Response::Json(str)) if str.starts_with('{') && str.ends_with('}') => Ok(()),
            Ok(Response::Json(str)) if str.starts_with("[{") && str.ends_with("}]") => Ok(()),
            _ => Err(format!("Expected JSON response, got: {response:?}")),
        }
    }
    let expect_json_warn = |response: &Result<Response, Error>| {
        expect_json(response).and_then(|_| match response {
            Ok(Response::Json(str)) if str.contains("warnings") => Ok(()),
            Ok(Response::Json(_)) => Err("Expected JSON with warnings".to_string()),
            _ => Err("Expected JSON response".to_string()),
        })
    };
    let expect_pbf = |response: &Result<Response, Error>| match response {
        Ok(Response::Pbf(_)) => Ok(()),
        _ => Err(format!("Expected PBF response, got: {response:?}")),
    };
    let expect_other = |response: &Result<Response, Error>| match response {
        Ok(Response::Other(_)) => Ok(()),
        _ => Err(format!("Expected binary response, got: {response:?}")),
    };

    struct EndpointTest {
        name: &'static str,
        endpoint: fn(&mut Actor, &proto::Options) -> Result<Response, Error>,
        options: proto::Options,
        format_checks: Vec<(Format, CheckFn)>,
    }

    let base_options = proto::Options {
        costing_type: proto::costing::Type::Auto as i32,
        locations: vec![
            proto::Location {
                ll: ANDORRA_TEST_LOC_1.into(),
                ..Default::default()
            },
            proto::Location {
                ll: ANDORRA_TEST_LOC_2.into(),
                ..Default::default()
            },
        ],
        ..Default::default()
    };

    let tests = vec![
        EndpointTest {
            name: "route",
            endpoint: Actor::route,
            options: base_options.clone(),
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                (Format::Pbf, expect_pbf),
                (Format::Gpx, expect_other),
                (Format::Geotiff, expect_json_warn),
            ],
        },
        EndpointTest {
            name: "locate",
            endpoint: Actor::locate,
            options: proto::Options {
                locations: vec![proto::Location {
                    ll: ANDORRA_TEST_LOC_1.into(),
                    ..Default::default()
                }],
                has_verbose: Some(proto::options::HasVerbose::Verbose(true)), // for more detailed output
                ..Default::default()
            },
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                // `locate` always returns JSON and doesn't support warnings
                (Format::Pbf, expect_json),
                (Format::Gpx, expect_json),
                (Format::Geotiff, expect_json),
            ],
        },
        EndpointTest {
            name: "matrix",
            endpoint: Actor::matrix,
            options: proto::Options {
                costing_type: proto::costing::Type::Auto as i32,
                sources: base_options.locations.clone(),
                targets: base_options.locations.clone(),
                ..Default::default()
            },
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                (Format::Pbf, expect_pbf),
                (Format::Gpx, expect_json_warn),
                (Format::Geotiff, expect_json_warn),
            ],
        },
        EndpointTest {
            name: "optimized_route",
            endpoint: Actor::optimized_route,
            options: base_options.clone(),
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                (Format::Pbf, expect_pbf),
                (Format::Gpx, expect_other),
                (Format::Geotiff, expect_json_warn),
            ],
        },
        EndpointTest {
            name: "isochrone",
            endpoint: Actor::isochrone,
            options: proto::Options {
                costing_type: proto::costing::Type::Auto as i32,
                locations: vec![proto::Location {
                    ll: ANDORRA_TEST_LOC_2.into(),
                    ..Default::default()
                }],
                contours: vec![proto::Contour {
                    has_time: Some(proto::contour::HasTime::Time(10.0)),
                    has_color: Some(proto::contour::HasColor::Color("ff0000".into())),
                    ..Default::default()
                }],
                ..Default::default()
            },
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                (Format::Pbf, expect_pbf),
                (Format::Gpx, expect_json_warn),
                (Format::Geotiff, expect_json_warn), // Geotiff requires GDAL
            ],
        },
        EndpointTest {
            name: "trace_route",
            endpoint: Actor::trace_route,
            options: proto::Options {
                costing_type: proto::costing::Type::Auto as i32,
                has_encoded_polyline: Some(proto::options::HasEncodedPolyline::EncodedPolyline(
                    "qwnapA__c|A_CeOu@qEyAkMs@cISuFEePS_Ze@yG_A}EwNyc@iG_P_BoE".into(),
                )),
                ..Default::default()
            },
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                (Format::Pbf, expect_pbf),
                (Format::Gpx, expect_other),
                (Format::Geotiff, expect_json_warn),
            ],
        },
        EndpointTest {
            name: "trace_attributes",
            endpoint: Actor::trace_attributes,
            options: proto::Options {
                costing_type: proto::costing::Type::Auto as i32,
                has_encoded_polyline: Some(proto::options::HasEncodedPolyline::EncodedPolyline(
                    "qwnapA__c|A_CeOu@qEyAkMs@cISuFEePS_Ze@yG_A}EwNyc@iG_P_BoE".into(),
                )),
                ..Default::default()
            },
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                (Format::Pbf, expect_pbf),
                (Format::Gpx, expect_json_warn),
                (Format::Geotiff, expect_json_warn),
            ],
        },
        EndpointTest {
            name: "transit_available",
            endpoint: Actor::transit_available,
            options: base_options.clone(),
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                // `transit_available` always returns JSON and none of them has warnings
                (Format::Pbf, expect_json),
                (Format::Gpx, expect_json),
                (Format::Geotiff, expect_json),
            ],
        },
        EndpointTest {
            name: "expansion",
            endpoint: Actor::expansion,
            options: proto::Options {
                action: proto::options::Action::Route as i32,
                has_expansion_action: Some(proto::options::HasExpansionAction::ExpansionAction(
                    proto::options::Action::Route as i32,
                )),
                ..base_options.clone()
            },
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                (Format::Pbf, expect_pbf),
                // no warnings for GPX and Geotiff, while they are not supported
                (Format::Gpx, expect_json),
                (Format::Geotiff, expect_json),
            ],
        },
        EndpointTest {
            name: "centroid",
            endpoint: Actor::centroid,
            options: base_options.clone(),
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json_warn),
                (Format::Pbf, expect_pbf),
                (Format::Gpx, expect_json_warn),
                (Format::Geotiff, expect_json_warn),
            ],
        },
        EndpointTest {
            name: "status",
            endpoint: Actor::status,
            options: proto::Options {
                has_verbose: Some(proto::options::HasVerbose::Verbose(true)),
                ..Default::default()
            },
            format_checks: vec![
                (Format::Json, expect_json),
                (Format::Osrm, expect_json),
                (Format::Pbf, expect_pbf),
                // no warnings for GPX and Geotiff, while they are not supported
                (Format::Gpx, expect_json),
                (Format::Geotiff, expect_json),
            ],
        },
    ];

    let config = ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: ANDORRA_TILES.into(),
            traffic_extract: ANDORRA_TRAFFIC.into(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    let mut actor = Actor::new(&config).unwrap();

    for test in tests {
        for (format, check) in test.format_checks {
            let request = proto::Options {
                format: format as i32,
                ..test.options.clone()
            };
            let response = (test.endpoint)(&mut actor, &request);
            assert_eq!(check(&response), Ok(()), "{:?} for {format:?}", test.name);
        }
    }
}

#[test]
fn parse_api() {
    assert!(Actor::parse_json_request("", proto::options::Action::Route).is_err());
    assert!(Actor::parse_json_request("{", proto::options::Action::Route).is_err());
    assert!(Actor::parse_json_request("}", proto::options::Action::Route).is_err());

    let json = r#"{"units":"kilometers","date_time":{"type":"current"},"costing":"auto","costing_options":{"auto":{"use_ferry":0.5,"use_rail_ferry":0.5,"use_highways":0.5,"use_tolls":0.5,"country_crossing_cost":0}},"alternates":2,"locations":[{"lat":42.50107335756198,"lon":1.510341967860551},{"lat":42.50627089323736,"lon":1.521734167223563}]}"#;
    let request = Actor::parse_json_request(json, proto::options::Action::Route)
        .expect("Failed to parse API");
    assert_eq!(request.units, proto::options::Units::Kilometers as i32);
    // assert_eq!(options.date_time.as_ref().unwrap().type_, proto::options::DateTimeType::Current as i32);
    assert_eq!(request.costing_type, proto::costing::Type::Auto as i32);
    assert_eq!(request.locations.len(), 2);
    assert_eq!(request.locations[0].ll, Some(ANDORRA_TEST_LOC_1.into()));
    assert_eq!(request.locations[1].ll, Some(ANDORRA_TEST_LOC_2.into()));
    assert_eq!(
        request.has_alternates,
        Some(proto::options::HasAlternates::Alternates(2))
    );

    let auto_costings = match request
        .costings
        .get(&(proto::costing::Type::Auto as i32))
        .and_then(|costing| costing.has_options.as_ref())
    {
        Some(proto::costing::HasOptions::Options(auto_costings)) => auto_costings,
        _ => panic!("Expected auto costing options to be present"),
    };
    assert_eq!(
        auto_costings.has_use_ferry,
        Some(proto::costing::options::HasUseFerry::UseFerry(0.5))
    );
    assert_eq!(
        auto_costings.has_use_rail_ferry,
        Some(proto::costing::options::HasUseRailFerry::UseRailFerry(0.5))
    );
    assert_eq!(
        auto_costings.has_use_highways,
        Some(proto::costing::options::HasUseHighways::UseHighways(0.5))
    );
    assert_eq!(
        auto_costings.has_use_tolls,
        Some(proto::costing::options::HasUseTolls::UseTolls(0.5))
    );
    assert_eq!(
        auto_costings.has_country_crossing_cost,
        Some(proto::costing::options::HasCountryCrossingCost::CountryCrossingCost(0.0))
    );

    // Parsed request should be routable
    let config = ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: ANDORRA_TILES.into(),
            traffic_extract: ANDORRA_TRAFFIC.into(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    let mut actor = Actor::new(&config).unwrap();
    let response = actor.route(&request);
    let Ok(Response::Json(_)) = response else {
        panic!("Expected JSON response, got: {response:?}");
    };
}
