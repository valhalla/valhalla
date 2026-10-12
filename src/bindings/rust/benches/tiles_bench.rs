use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;
use valhalla::{Config, ConfigBuilder, GraphReader, LiveTraffic, TileId};

fn edgeinfo(c: &mut Criterion) {
    let config = Config::from_tile_extract("./tests/andorra/tiles.tar").unwrap();
    let reader = GraphReader::new(&config).unwrap();
    let tiles: Vec<_> = reader
        .tiles()
        .into_iter()
        .map(|id| reader.graph_tile(id).unwrap())
        .collect();

    c.bench_function("way_id over the tileset", |b| {
        b.iter(|| {
            let mut sum = 0u64;
            for tile in &tiles {
                for de in tile.directededges() {
                    sum += black_box(tile.edgeinfo(de)).way_id;
                }
            }
            black_box(sum)
        })
    });

    c.bench_function("edge shapes over the tileset", |b| {
        b.iter(|| {
            let mut points_count = 0;
            for tile in &tiles {
                for de in tile.directededges() {
                    points_count += black_box(tile.edgeinfo(de).shape().len());
                }
            }
            black_box(points_count)
        })
    });
}

fn write_traffic(c: &mut Criterion) {
    let config = ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: "./tests/andorra/tiles.tar".to_string(),
            traffic_extract: "./tests/andorra/traffic.tar".to_string(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    let graph_reader = GraphReader::new(&config).unwrap();

    // find a tile the the most edges
    let mut max_edges = 0;
    let mut max_tile_id = TileId::default();
    for tile_id in graph_reader.tiles() {
        let tile = graph_reader.graph_tile(tile_id).unwrap();

        let edge_count = tile.directededges().len();
        if edge_count > max_edges {
            max_edges = edge_count;
            max_tile_id = tile_id;
        }
    }

    let traffic_tile = graph_reader.traffic_tile(max_tile_id).unwrap();
    c.bench_function("write live traffic", |b| {
        b.iter(|| {
            for i in 0..max_edges {
                let traffic = LiveTraffic::from_uniform_speed((10 + i % 100) as u8);
                traffic_tile.write_edge_traffic(i as u32, black_box(traffic));
            }
        });
    });

    c.bench_function("read live traffic", |b| {
        b.iter(|| {
            for i in 0..max_edges {
                let traffic = traffic_tile.edge_traffic(i as u32);
                black_box(traffic);
            }
        });
    });

    c.bench_function("clear live traffic", |b| {
        b.iter(|| {
            traffic_tile.clear_traffic();
        });
    });
}

criterion_group!(benches, edgeinfo, write_traffic);
criterion_main!(benches);
