#![cfg(feature = "proto")]

use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;
use valhalla::{Actor, ConfigBuilder, LatLon, proto};

const UTRECHT_TILES: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/build/test/data/utrecht_tiles/tiles.tar"
);
const UTRECHT_TRAFFIC: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/build/test/data/utrecht_tiles/traffic.tar"
);

const UTRECHT_TEST_LOC_1: LatLon = LatLon(52.08946, 5.11006); // Utrecht Centraal
const UTRECHT_TEST_LOC_2: LatLon = LatLon(52.09068, 5.12136); // Dom Tower

fn route(c: &mut Criterion) {
    let config = ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: UTRECHT_TILES.into(),
            traffic_extract: UTRECHT_TRAFFIC.into(),
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
                    ll: UTRECHT_TEST_LOC_1.into(),
                    ..Default::default()
                },
                proto::Location {
                    ll: UTRECHT_TEST_LOC_2.into(),
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
                    ll: UTRECHT_TEST_LOC_1.into(),
                    ..Default::default()
                },
                proto::Location {
                    ll: UTRECHT_TEST_LOC_2.into(),
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
                    ll: LatLon(52.08813, 5.03231).into(),
                    ..Default::default()
                },
                proto::Location {
                    ll: LatLon(52.09987, 5.14913).into(),
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
                    ll: LatLon(52.08813, 5.03231).into(),
                    ..Default::default()
                },
                proto::Location {
                    ll: LatLon(52.09987, 5.14913).into(),
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
            tile_extract: UTRECHT_TILES.into(),
            traffic_extract: UTRECHT_TRAFFIC.into(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    let mut actor = Actor::new(&config).unwrap();

    // shape of the "long route" bench
    let shape = "}sejbBqvcrHtFkuA|HgkBnFexA~Ay`@nB_[tAwZxBey@TwPGuOw@qMcBqLeCsKsDuJ}G}LqHmJmIyIyCqDb@{BHiCMgCg@yB}@aBkA{@sASsATkA~@{@bBeByB}CeEgEcHgHaQmEgOsJi^mA{EbAyAn@wBTiCEmC_@eCg@sAq@cA{@m@_AUg@wE]uF]aONeLd@gKtCqZfEu_@zE}[zDsUrEkVzDeQzCmHzAiClCiCpEaBbFm@hY`D~Fl@fE^bk@lFhZlCfFKnCa@fEs@nA~A~Ah@`BW|A_Bz@uCLqDdCwElD_M`D_Yt@sKf@_TXmSH}RDgM?aF`BAjAGzHm@lOaAdIw@xFYxK@tI`@xMvApb@pFnUzDfc@vIze@fM|[bJvR|GpYtLdVbK~J|DvEnBtEtB~g@`VtdA|b@dCjAbNhHnFhC~h@pS|GpDNnCb@dCt@rBbAxApA|@zA\\zAGvAk@nAoA~@kB~GbCxCdAnQjH|GdCxO|E`FfBfk@fU|kBpu@z}@h^nFrBbCbAvDjDRjDt@nCfA|AlAr@{_A|gLcUnwBiPj~AkQn{AcApKsArNhGlDvYlQ`NfK~_@d\\vDlAjBTvAGnA}@fBqCx@oEjAqKp@sKlIq|B|Ke_CtAwYx[uaFnAgm@t`CmhY|C_^rO_aBxbAwkHdf@_~ClM{z@nnB_bMbtAqrInIue@zLqg@pO}k@dUwq@xf@kwAjLs`@zIeb@tFc^vDu]zCqb@tAy}AQkm@Twk@|Aap@tDqcAtCmw@jd@_aLj[{vHbY_vFvG}xAzFu|@vL}sAn\\koCrLw`Avg@qzDha@unBhW}qA`TwfA`SueA|JcYnOc]xI}JzKuYrEq[tAwWEmRsAoNkBqPwIkW{GcOsLoK{JkDgHiA}E{@wEScLtBaKhFaKtJ}JxHwK~IuMvEoSp@}Fw@oNaCugA}_@_w@uYcmAme@c}A_m@w|@c_@_zAul@w\\sRy^}Vk`@a]eYeYs_@ud@u]og@wTsY_Xs[kh@{s@iSu[oMsTiQg]w[mt@uMa]mXcv@aQsg@}g@_|A_My^mOg^eQ_\\gOqUcN}OuUmTcMiJeLwH}I{FkIoFkWmOmlAcu@aLwHauBkuA{d@sZqbBmeAmo@oa@uc@w\\c[c[inBe{B}JuJsQoQgQ_NmDkBgCuAuH_EuQwFmJgAaH_AuUh@gPzA_P~FmV|Kw^bWuK`K_a@j`@mQdNkQ`NcPtK_XrOyIxEcRvFaEhAoHdAifD|hAyGfCoW|Ims@fVgNnEcFfAiDG_HoBwA_AwE{CqCyBoDgAuD@mDjA_DnCeBrCuArD}@hEiFzIuDxE_H`FuUhHcUfGaj@hKmc@~Hq\\rAyd@e@eMWmVuCg\\yBu]}BuEg@mFqBeEmBuDyBiDyNyBsDmBuMuAcLWoMdBklA[}h@mBqe@fCiA|Dk@pFMlTa@zAF`@yHnFqg@fEw`@zCgZvBn@|Cz@rA^~K~C|WpH";

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
            tile_extract: UTRECHT_TILES.into(),
            traffic_extract: UTRECHT_TRAFFIC.into(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    let mut actor = Actor::new(&config).unwrap();

    c.bench_function("locate", |b| {
        let request = proto::Options {
            locations: vec![proto::Location {
                ll: UTRECHT_TEST_LOC_1.into(),
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
                "lat":52.08946,
                "lon":5.11006
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
            tile_extract: UTRECHT_TILES.into(),
            traffic_extract: UTRECHT_TRAFFIC.into(),
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
