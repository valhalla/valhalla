use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;
use valhalla::ConfigBuilder;

const UTRECHT_TILES: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/build/test/data/utrecht_tiles/tiles.tar"
);
const UTRECHT_TRAFFIC: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/build/test/data/utrecht_tiles/traffic.tar"
);

fn config_construction(c: &mut Criterion) {
    c.bench_function("config builder (full utrecht config)", |b| {
        b.iter(|| {
            let builder = ConfigBuilder {
                mjolnir: valhalla::config::Mjolnir {
                    tile_extract: UTRECHT_TILES.to_string(),
                    traffic_extract: UTRECHT_TRAFFIC.to_string(),
                    ..Default::default()
                },
                ..Default::default()
            };
            black_box(builder.build())
        })
    });

    c.bench_function("config builder (defaults only)", |b| {
        b.iter(|| black_box(valhalla::ConfigBuilder::default().build()))
    });
}

criterion_group!(benches, config_construction);
criterion_main!(benches);
