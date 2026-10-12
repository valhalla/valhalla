#![cfg(feature = "proto")]

use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;
use valhalla::{Actor, ConfigBuilder, LatLon, proto};

const ANDORRA_TILES: &str = "tests/andorra/tiles.tar";
const ANDORRA_TRAFFIC: &str = "tests/andorra/traffic.tar";

const ANDORRA_TEST_LOC_1: LatLon = LatLon(42.50107335756198, 1.510341967860551); // Sant Julia de Loria
const ANDORRA_TEST_LOC_2: LatLon = LatLon(42.50627089323736, 1.521734167223563); // Andorra la Vella

fn route(c: &mut Criterion) {
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

    c.bench_function("short route json", |b| {
        let request = proto::Options {
            format: proto::options::Format::Json as i32,
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

        b.iter(|| {
            let response = actor.route(black_box(&request)).unwrap();
            black_box(response)
        });
    });

    c.bench_function("short route pbf", |b| {
        let request = proto::Options {
            format: proto::options::Format::Pbf as i32,
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

        b.iter(|| {
            let response = actor.route(black_box(&request)).unwrap();
            black_box(response)
        });
    });

    c.bench_function("long route json", |b| {
        let request = proto::Options {
            format: proto::options::Format::Json as i32,
            costing_type: proto::costing::Type::Auto as i32,
            locations: vec![
                proto::Location {
                    ll: LatLon(42.54381401912126, 1.4756460643803673).into(),
                    ..Default::default()
                },
                proto::Location {
                    ll: LatLon(42.54262715333714, 1.7332292461658099).into(),
                    ..Default::default()
                },
            ],
            ..Default::default()
        };

        b.iter(|| {
            let response = actor.route(black_box(&request)).unwrap();
            black_box(response)
        });
    });

    c.bench_function("long route pbf", |b| {
        let request = proto::Options {
            format: proto::options::Format::Pbf as i32,
            costing_type: proto::costing::Type::Auto as i32,
            locations: vec![
                proto::Location {
                    ll: LatLon(42.54381401912126, 1.4756460643803673).into(),
                    ..Default::default()
                },
                proto::Location {
                    ll: LatLon(42.54262715333714, 1.7332292461658099).into(),
                    ..Default::default()
                },
            ],
            ..Default::default()
        };

        b.iter(|| {
            let response = actor.route(black_box(&request)).unwrap();
            black_box(response)
        });
    });
}

fn trace_attributes(c: &mut Criterion) {
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

    let shape = "ifadpAyon{A`ClBtAg@~FwKbOwIeAtL[lJt@hDnCtEdB~D|BnDbAj@v@Sr@s@?sAy@gIkA_ICyDh@{Cz@}At@k@fACbAb@j@~@xAvHvHn`@p@pE\\~@t@`@lAoA\\wCo@aRu@uSOoG`@mBr@oAbBw@dBNxA|Bj@|HbDj\\f@lBt@f@fAIr@uCtBaPbCaSj@eD~A{AjCk@nBrBb@jDyJr`AG~DnBzCbChAb@YbAo@bA}FzGmo@pAuHt@iC~AwBhAq@dBIbH~ApCXrBA`Ba@bBz@MvCeB`CeB`A}@f@yDjD}DlEsAlDE|AXrAr@j@|@OnAqAtBmFzAcC|C_@zEg@l@^P~@S|Bo@hA_B|@wBlAmAd@]fBTjCOlJqAbKx@rGFtCN~FcBjIqDxIwBlD?zBGzCg@Rc@^Yl@Ot@Cd@?f@Df@Jb@P^TZVTZLZD\\?ZGXOTSRYN]J_@Fc@|GwAbCG|DCnA@tDf@xCGvI_CfBu@~@i@ZQ|DyBtIeF|BuAdCoERkACoBUoBcBsDyBoCiB_C}@kBuAcEu@aDQw@]uCKsB@qAPkB^cC~@}EdBgGrUih@vAaDjEaJlMqYrMmY|@uA`AiAnDwDtDiDvBiCX[`BoBtDgFjB_ENg@Bk@x@uIb@wG`@eIJyAzAar@^qQ_AsUkCuRy@aGuE}PyDqLyFaJiDiFyMgRsImN_AuAeBsB}ByAkDy@mFe@oEYqF}AsHcDeB{@aA[g@M{Cy@gA[{Bm@{IyBkGaB{EgBmB{@aB}@mCoBgCoB}HaJmSmV}QmUqD{EeDgG}BsFoB{I}@sF_DaYu@yEs@mEs@qCw@eC_DuHeDaIaAaCwEgKkFgL}JkUwHoOqBuDqBuCcB}B_CyBuFeEmDoBwBqA_G_CsFyA_@IoEy@sEk@_DMuAKmBWy@a@w@y@Qq@E}FFW`@eB|@wAbA{@tAc@rBB`DvB~TpNhKtGlSvM|GhEjPlH`NpF";

    c.bench_function("trace attributes edge walk json", |b| {
        let request = proto::Options {
            costing_type: proto::costing::Type::Auto as i32,
            shape_match: proto::ShapeMatch::EdgeWalk as i32,
            has_encoded_polyline: Some(proto::options::HasEncodedPolyline::EncodedPolyline(
                shape.into(),
            )),
            ..Default::default()
        };
        b.iter(|| {
            let response = actor.trace_attributes(black_box(&request)).unwrap();
            black_box(response)
        });
    });

    c.bench_function("trace attributes edge walk pbf", |b| {
        let request = proto::Options {
            costing_type: proto::costing::Type::Auto as i32,
            shape_match: proto::ShapeMatch::EdgeWalk as i32,
            has_encoded_polyline: Some(proto::options::HasEncodedPolyline::EncodedPolyline(
                shape.into(),
            )),
            format: proto::options::Format::Pbf as i32,
            ..Default::default()
        };
        b.iter(|| {
            let response = actor.trace_attributes(black_box(&request)).unwrap();
            black_box(response)
        });
    });

    c.bench_function("trace attributes map snap json", |b| {
        let request = proto::Options {
            costing_type: proto::costing::Type::Auto as i32,
            shape_match: proto::ShapeMatch::MapSnap as i32,
            has_encoded_polyline: Some(proto::options::HasEncodedPolyline::EncodedPolyline(
                shape.into(),
            )),
            ..Default::default()
        };
        b.iter(|| {
            let response = actor.trace_attributes(black_box(&request)).unwrap();
            black_box(response)
        });
    });

    c.bench_function("trace attributes map snap pbf", |b| {
        let request = proto::Options {
            costing_type: proto::costing::Type::Auto as i32,
            shape_match: proto::ShapeMatch::MapSnap as i32,
            has_encoded_polyline: Some(proto::options::HasEncodedPolyline::EncodedPolyline(
                shape.into(),
            )),
            format: proto::options::Format::Pbf as i32,
            ..Default::default()
        };
        b.iter(|| {
            let response = actor.trace_attributes(black_box(&request)).unwrap();
            black_box(response)
        });
    });
}

fn locate(c: &mut Criterion) {
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

    c.bench_function("locate", |b| {
        let request = proto::Options {
            locations: vec![proto::Location {
                ll: ANDORRA_TEST_LOC_1.into(),
                ..Default::default()
            }],
            ..Default::default()
        };

        b.iter(|| {
            let response = actor.locate(&request).unwrap();
            black_box(response)
        });
    });

    c.bench_function("locate json", |b| {
        let request = r#"{
            "locations":[{
                "lat":42.50107335756198,
                "lon":1.510341967860551
            }],
            "verbose": true
        }"#;
        b.iter(|| {
            let request =
                Actor::parse_json_request(black_box(request), proto::options::Action::Locate)
                    .unwrap();
            let response = actor.locate(&request).unwrap();
            black_box(response)
        });
    });
}

fn status(c: &mut Criterion) {
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

    c.bench_function("status", |b| {
        let request = proto::Options {
            has_verbose: Some(proto::options::HasVerbose::Verbose(true)),
            ..Default::default()
        };

        b.iter(|| {
            let response = actor.status(&request).unwrap();
            black_box(response)
        });
    });
}

criterion_group!(benches, route, trace_attributes, locate, status);
criterion_main!(benches);
