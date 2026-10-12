use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;
use valhalla::ConfigBuilder;

fn config_construction(c: &mut Criterion) {
    c.bench_function("config builder (full andorra config)", |b| {
        b.iter(|| {
            let builder = ConfigBuilder {
                mjolnir: valhalla::config::Mjolnir {
                    tile_extract: "./tests/andorra/tiles.tar".to_string(),
                    traffic_extract: "./tests/andorra/traffic.tar".to_string(),
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
