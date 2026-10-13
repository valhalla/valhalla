use pretty_assertions::assert_eq;

use valhalla::{
    Access, Config, ConfigBuilder, GraphId, GraphReader, LatLon, LiveTraffic, NodeType, TileId,
    TimeZoneInfo, level,
};

mod common;
use common::utrecht_tiles;

const UTRECHT_BBOX: (LatLon, LatLon) = (LatLon(52.04, 5.01), LatLon(52.15, 5.17));

// The only edge in test/data/traffic_tiles/0/003/196.csv
fn historical_traffic_edge() -> GraphId {
    GraphId::from_parts(level::HIGHWAY, 3196, 1155).unwrap()
}

fn utrecht_reader(traffic_extract: &str) -> GraphReader {
    let config = ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: utrecht_tiles("tiles.tar"),
            traffic_extract: traffic_extract.into(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    GraphReader::new(&config).expect("Failed to create GraphReader")
}

#[test]
fn dataset_id() {
    let reader = utrecht_reader(&utrecht_tiles("traffic.tar"));

    assert_eq!(reader.dataset_id(), 34979400);
}

#[test]
fn graph_tile_can_outlive_reader() {
    let reader = utrecht_reader(&utrecht_tiles("traffic.tar"));

    let tile = reader.graph_tile(reader.tiles()[0]).unwrap();
    // just do something complicated with the tile
    let count = tile
        .directededges()
        .iter()
        .filter(|e| tile.edgeinfo(e).speed_limit == 0)
        .count();
    assert_ne!(count, 0);

    // Dropping reader should not affect the tile and its data
    drop(reader);

    let other_count = tile
        .directededges()
        .iter()
        .filter(|e| tile.edgeinfo(e).speed_limit == 0)
        .count();
    assert_eq!(
        count, other_count,
        "Tile should remain valid after reader is dropped"
    );

    // Sanity check that clone works as expected
    let cloned_tile = tile.clone();
    drop(tile);

    let cloned_count = cloned_tile
        .directededges()
        .iter()
        .filter(|e| cloned_tile.edgeinfo(e).speed_limit == 0)
        .count();
    assert_eq!(
        count, cloned_count,
        "Clone should remain valid after the tile is dropped"
    );
}

#[test]
fn traffic_tile_can_outlive_reader() {
    // the C++ tests use the same traffic.tar, so write to a copy
    let traffic_copy = tempfile::NamedTempFile::new().unwrap();
    std::fs::copy(utrecht_tiles("traffic.tar"), traffic_copy.path()).unwrap();
    let reader = utrecht_reader(traffic_copy.path().to_str().unwrap());

    let tile = reader.traffic_tile(reader.tiles()[0]).unwrap();
    tile.write_spare(1);
    assert_eq!(tile.spare(), 1);

    // Dropping reader should not affect the tile and its data
    drop(reader);

    assert_eq!(
        tile.spare(),
        1,
        "Tile should remain valid after reader is dropped"
    );
}

#[test]
fn graph_tile_accepts_any_id_inside_the_tile() {
    let reader = utrecht_reader(&utrecht_tiles("traffic.tar"));

    for tile_id in reader.tiles() {
        let tile = reader.graph_tile(tile_id).unwrap();
        let last_edge = (tile.directededges().len() as u32).saturating_sub(1);
        let last_node = (tile.nodes().len() as u32).saturating_sub(1);
        for id in [0, last_edge, last_node] {
            let id = tile_id.graph_id(id).unwrap();
            assert_eq!(reader.graph_tile(id).unwrap().id(), tile_id, "{id}");
            assert_eq!(reader.traffic_tile(id).unwrap().id(), tile_id, "{id}");
        }
    }
}

#[test]
fn tiles_in_bbox() {
    let reader = utrecht_reader(&utrecht_tiles("traffic.tar"));

    assert!(reader.graph_tile(GraphId::default()).is_none());
    assert!(reader.traffic_tile(GraphId::default()).is_none());

    let mut all_tiles = reader.tiles();
    all_tiles.sort(); // order is not guaranteed, sort for comparison
    assert!(!all_tiles.is_empty());

    let mut world_tiles: Vec<TileId> = (level::HIGHWAY..=level::LOCAL)
        .flat_map(|level| reader.tiles_in_bbox(LatLon(-90.0, -180.0), LatLon(90.0, 180.0), level))
        .collect();
    world_tiles.sort();
    assert_eq!(all_tiles, world_tiles);

    let mut bbox_tiles: Vec<TileId> = (level::HIGHWAY..=level::LOCAL)
        .flat_map(|level| reader.tiles_in_bbox(UTRECHT_BBOX.0, UTRECHT_BBOX.1, level))
        .collect();
    bbox_tiles.sort();
    assert_eq!(all_tiles, bbox_tiles);

    // And no other levels
    for level in [3, 7, u8::MAX] {
        let tiles = reader.tiles_in_bbox(LatLon(-90.0, -180.0), LatLon(90.0, 180.0), level);
        assert!(tiles.is_empty());
    }

    for level in level::HIGHWAY..=level::LOCAL {
        let tiles = reader.tiles_in_bbox(UTRECHT_BBOX.0, UTRECHT_BBOX.1, level);
        assert!(!tiles.is_empty(), "level {level}");
        for tile_id in tiles {
            assert_ne!(tile_id, TileId::default());
            assert_eq!(tile_id.level(), level);

            let tile = reader.graph_tile(tile_id).unwrap();
            assert_eq!(tile.id(), tile_id);

            let traffic_tile = reader.traffic_tile(tile_id).unwrap();
            assert_eq!(traffic_tile.id(), tile_id);
            assert_eq!(
                traffic_tile.edge_count() as usize,
                tile.directededges().len()
            );
        }
    }

    let tile = reader.graph_tile(reader.tiles()[0]).unwrap();
    // The first admin info is always empty (for ocean)
    let admininfo = tile.admin_info(0).unwrap();
    assert_eq!(admininfo.country_iso, "");
    assert_eq!(admininfo.state_iso, "");
    assert_eq!(admininfo.country_text, "None");
    assert_eq!(admininfo.state_text, "None");

    // The second admin info is the Netherlands
    let admininfo = tile.admin_info(1).unwrap();
    assert_eq!(admininfo.country_iso, "NL");
    assert_eq!(admininfo.state_iso, "");
    assert_eq!(admininfo.country_text, "Nederland");
    assert_eq!(admininfo.state_text, "");

    // There is no third admin info in this tileset
    assert!(tile.admin_info(2).is_none());
}

#[test]
fn edges_in_tile() {
    let reader = utrecht_reader(&utrecht_tiles("traffic.tar"));

    let historical_edge = historical_traffic_edge();
    for tile_id in reader.tiles() {
        let tile = reader.graph_tile(tile_id).unwrap();

        let slice = tile.directededges();
        assert!(!slice.is_empty());
        for (i, de) in slice.iter().enumerate() {
            // Ensure that the directed edge index matches the slice index.
            // This assertion ensures that the pointer arithmetic in the Rust FFI is correct.
            let via_ptr = tile.directededge(i as u32).unwrap();
            assert_eq!(de as *const _, via_ptr as *const _);

            assert_ne!(de.speed(), 0, "Default edge's speed should never be zero");
            assert_eq!(tile.live_traffic(de).speed(), None);
            if tile_id.graph_id(i as u32) != Some(historical_edge) {
                assert_eq!(de.free_flow_speed(), 0);
                assert_eq!(de.constrained_flow_speed(), 0);
                assert_eq!(
                    tile.edge_speed(de, valhalla::SpeedSources::ALL, false, 0, 0),
                    (de.speed(), valhalla::SpeedSources::NO_FLOW)
                );
                let truck_speed = if de.truck_speed() > 0 {
                    de.truck_speed()
                } else {
                    de.speed()
                };
                assert_eq!(
                    tile.edge_speed(de, valhalla::SpeedSources::ALL, true, 0, 0),
                    (truck_speed, valhalla::SpeedSources::NO_FLOW)
                );
            }

            let ei = tile.edgeinfo(de);
            assert_eq!(de.is_shortcut(), ei.way_id == 0, "Shortcuts have way_id 0");

            let endnode = de.endnode();
            assert_eq!(de.leaves_tile(), de.endnode().tile() != tile_id);
            if de.leaves_tile() {
                assert!(reader.graph_tile(endnode.tile()).is_some());
            }
        }
        assert!(tile.directededge(slice.len() as u32).is_none());
    }
}

#[test]
fn edge_with_historical_traffic() {
    let reader = GraphReader::new(&Config::from_tile_extract(utrecht_tiles("tiles.tar")).unwrap())
        .expect("Failed to create GraphReader");
    let id = historical_traffic_edge();
    let tile = reader.graph_tile(id).unwrap();
    let de = tile.directededge(id.id()).unwrap();

    assert_eq!(de.free_flow_speed(), 45);
    assert_eq!(de.constrained_flow_speed(), 35);
    let (_, sources) = tile.edge_speed(de, valhalla::SpeedSources::ALL, false, 0, 0);
    assert_eq!(sources, valhalla::SpeedSources::PREDICTED_FLOW);
    // free flow is used at night and constrained flow during the day
    assert_eq!(
        tile.edge_speed(de, valhalla::SpeedSources::FREE_FLOW, false, 0, 0),
        (45, valhalla::SpeedSources::FREE_FLOW)
    );
    assert_eq!(
        tile.edge_speed(
            de,
            valhalla::SpeedSources::CONSTRAINED_FLOW,
            false,
            12 * 3600,
            0
        ),
        (35, valhalla::SpeedSources::CONSTRAINED_FLOW)
    );
}

#[test]
fn live_traffic_without_traffic_extract() {
    let reader = utrecht_reader("bad_path_to_traffic_extract");

    for tile_id in reader.tiles() {
        assert!(reader.traffic_tile(tile_id).is_none());
    }

    let tile_id = reader.tiles()[0];
    let tile = reader.graph_tile(tile_id).unwrap();
    let edges = tile.directededges();
    assert!(!edges.is_empty());
    for de in edges {
        let live = tile.live_traffic(de);
        assert_eq!(live.speed(), None);
        // INVALID_SPEED from traffictile.h, not a zeroed record
        assert_eq!(live.to_bits(), 0x0FFF_FFFF);
    }
}

#[test]
fn nodes_in_tile() {
    let reader = utrecht_reader(&utrecht_tiles("traffic.tar"));

    let mut no_auto_access_count = 0;
    let mut transition_count = 0;
    let mut gate_count = 0;
    let mut bollard_count = 0;
    let mut private_access_count = 0;
    for tile_id in reader.tiles() {
        let tile = reader.graph_tile(tile_id).unwrap();

        let slice = tile.nodes();
        assert!(!slice.is_empty(), "Tile should always have nodes");
        for (i, node) in slice.iter().enumerate() {
            // Ensure that the node index matches the slice index.
            // This assertion ensures that the pointer arithmetic in the Rust FFI is correct.
            let via_ptr = tile.node(i as u32).unwrap();
            assert_eq!(node as *const _, via_ptr as *const _);

            // Check no panic
            let _ = tile.node_edges(node);
            let transitions = tile.node_transitions(node);
            transition_count += transitions.len();

            assert_eq!(node.admin_index(), 1); // Netherlands only, see [`tiles_in_bbox`] test

            // Europe/Brussels, tzdata links Europe/Amsterdam to it
            assert_eq!(node.timezone(), 298);

            // All nodes should be within Utrecht bbox
            let ll = tile.node_latlon(node);
            assert!(ll.0 >= UTRECHT_BBOX.0.0 && ll.0 <= UTRECHT_BBOX.1.0);
            assert!(ll.1 >= UTRECHT_BBOX.0.1 && ll.1 <= UTRECHT_BBOX.1.1);

            // This tileset has no elevation data
            assert_eq!(node.elevation(), -500.0);

            if !node.access().intersects(Access::AUTO) {
                no_auto_access_count += 1;
            }

            match node.node_type() {
                NodeType::kGate => gate_count += 1,
                NodeType::kBollard => bollard_count += 1,
                _ => {}
            }
            if node.private_access() {
                // Access mask stays fully open, `private_access()` is the only way to spot it.
                assert!(node.access().contains(Access::AUTO));
                private_access_count += 1;
            }
        }
    }
    assert!(transition_count > 0);
    assert!(no_auto_access_count > 0);
    assert!(gate_count > 0);
    assert!(bollard_count > 0);
    assert!(private_access_count > 0);
}

#[test]
fn reverse_edge() {
    let reader = utrecht_reader(&utrecht_tiles("traffic.tar"));

    let tile_id = reader.tiles()[0];
    let tile = reader.graph_tile(tile_id).unwrap();

    for (de_index, de) in tile.directededges().iter().enumerate() {
        if de.leaves_tile() || de.is_shortcut() {
            // just don't bother with such edges for this test
            continue;
        }

        let end_node = tile.node(de.endnode().id()).unwrap();
        let opp_de = &tile.node_edges(end_node)[de.opp_index() as usize];
        assert_eq!(tile.edgeinfo(de).way_id, tile.edgeinfo(opp_de).way_id);

        let begin_node = tile.node(opp_de.endnode().id()).unwrap();
        assert_eq!(
            de_index,
            (begin_node.edge_index() + opp_de.opp_index()) as usize
        );
    }
}

#[test]
fn live_traffic() {
    // for this test we should work with copy of the traffic tar to avoid modifying the original one
    let traffic_copy = tempfile::NamedTempFile::new().unwrap();
    std::fs::copy(utrecht_tiles("traffic.tar"), traffic_copy.path())
        .expect("Failed to copy traffic tar");

    let reader = utrecht_reader(traffic_copy.path().to_str().unwrap());

    // find a tile with the most edges to have a good number of edges to test
    let tile_id = {
        let mut max_edges = 0;
        let mut max_tile_id = TileId::default();
        for tile_id in reader.tiles() {
            let tile = reader.graph_tile(tile_id).unwrap();

            let edge_count = tile.directededges().len();
            if edge_count > max_edges {
                max_edges = edge_count;
                max_tile_id = tile_id;
            }
        }
        assert!(max_edges > 100);
        max_tile_id
    };
    let tile = reader.graph_tile(tile_id).unwrap();
    let traffic_tile = reader.traffic_tile(tile_id).unwrap();
    assert_eq!(
        tile.directededges().len(),
        traffic_tile.edge_count() as usize
    );

    assert_eq!(traffic_tile.last_update(), 0); // initial state
    traffic_tile.write_last_update(101);
    assert_eq!(traffic_tile.last_update(), 101);
    traffic_tile.write_spare(999);
    assert_eq!(traffic_tile.spare(), 999);
    traffic_tile.clear_traffic();
    assert_eq!(traffic_tile.last_update(), 0);
    assert_eq!(traffic_tile.spare(), 999); // spare is not cleared

    for edge_id in 0..traffic_tile.edge_count() {
        let edge = tile.directededge(edge_id).unwrap();

        // no traffic data in the tileset
        assert_eq!(
            traffic_tile.edge_traffic(edge_id),
            Some(LiveTraffic::UNKNOWN)
        );
        assert_eq!(tile.live_traffic(edge).speed(), None);

        traffic_tile.write_edge_traffic(edge_id, LiveTraffic::CLOSED);
        assert_eq!(tile.live_traffic(edge).speed(), Some(0)); // Some(0) = closed

        traffic_tile.write_edge_traffic(edge_id, LiveTraffic::from_uniform_speed(72));
        assert_eq!(tile.live_traffic(edge).speed(), Some(72));

        // speed is stored with 2km/h precision
        traffic_tile.write_edge_traffic(edge_id, LiveTraffic::from_uniform_speed(73));
        assert_eq!(tile.live_traffic(edge).speed(), Some(72));

        // spare bit changes nothing
        traffic_tile.write_edge_traffic(
            edge_id,
            LiveTraffic::from_uniform_speed(72).with_spare(true),
        );
        assert_eq!(tile.live_traffic(edge).speed(), Some(72));
        assert!(tile.live_traffic(edge).spare());

        // Only "overall speed" affects `speed()`. Per-segment detail is readable via `segments()`.
        traffic_tile.write_edge_traffic(
            edge_id,
            LiveTraffic::from_segmented_speeds(72, [1, 2, 3], [127, 128]),
        );
        let live = tile.live_traffic(edge);
        assert_eq!(live.speed(), Some(72));
        let segments: Vec<_> = live.segments().collect();
        assert_eq!(segments.len(), 3);
        // encoded speeds are stored with 2km/h precision (value >> 1 on write, << 1 on read);
        // 1 km/h rounds down to encoded 0, which reads back as a closed segment (0).
        assert_eq!(segments[0].speed, 0);
        assert_eq!(segments[1].speed, 2);
        assert_eq!(segments[2].speed, 2);
        // breakpoints [127, 128] become contiguous fractional ranges ending at 1.0.
        assert_eq!(segments[0].range, (0.0, 127.0 / 255.0));
        assert_eq!(segments[1].range, (127.0 / 255.0, 128.0 / 255.0));
        assert_eq!(segments[2].range, (128.0 / 255.0, 1.0));

        // congestion / has_incidents / spare round-trip through the GraphTile accessor
        let record = LiveTraffic::from_segmented_speeds(72, [60, 80, 100], [100, 200])
            .with_congestion([Some(0.0), Some(0.5), None])
            .with_incidents(true)
            .with_spare(true);
        traffic_tile.write_edge_traffic(edge_id, record);
        let live = tile.live_traffic(edge);
        let congestion: Vec<_> = live.segments().map(|segment| segment.congestion).collect();
        assert_eq!(congestion.len(), 3);
        assert_eq!(congestion[0], Some(0.0));
        assert!((congestion[1].unwrap() - 0.5).abs() < 0.02);
        assert_eq!(congestion[2], None);
        assert!(live.has_incidents());
        assert!(live.spare());
    }
}

#[test]
fn historical_traffic() {
    // bad slices should be rejected
    assert!(valhalla::encode_weekly_speeds(&[]).is_err());
    assert!(valhalla::encode_weekly_speeds(&[50.0]).is_err());
    assert!(valhalla::encode_weekly_speeds(&[50.0; 3000]).is_err());

    let speeds = vec![50.0; 2016];
    let encoded = valhalla::encode_weekly_speeds(&speeds).expect("Failed to encode speeds");
    assert!(!encoded.is_empty());

    // For flat speed values the error from lossy compression should be minimal
    let decoded = valhalla::decode_weekly_speeds(&encoded).expect("Failed to decode speeds");
    assert_eq!(decoded.len(), speeds.len());
    for (a, b) in decoded.iter().zip(speeds.iter()) {
        assert!(
            (a - b).abs() < 0.001, // for km/h 1e-3 is good enough
            "Decoded speed does not match original - {a} vs {b}"
        );
    }
}

#[test]
fn tz_info() {
    // Summer
    let unix_timestamp = 1750000000; // Jun 15 2025
    assert!(TimeZoneInfo::from_id(0, unix_timestamp).is_none());

    let tz = TimeZoneInfo::from_id(293, unix_timestamp).unwrap();
    assert_eq!(tz.name, "Europe/Andorra");
    assert_eq!(tz.offset_seconds, 7200); // UTC+2

    let tz = TimeZoneInfo::from_id(94, unix_timestamp).unwrap();
    assert_eq!(tz.name, "America/Los_Angeles");
    assert_eq!(tz.offset_seconds, -25200); // UTC-7

    // Winter
    let unix_timestamp = 1740000000; // Feb 19 2025
    assert!(TimeZoneInfo::from_id(0, unix_timestamp).is_none());

    let tz = TimeZoneInfo::from_id(293, unix_timestamp).unwrap();
    assert_eq!(tz.name, "Europe/Andorra");
    assert_eq!(tz.offset_seconds, 3600); // UTC+1

    let tz = TimeZoneInfo::from_id(94, unix_timestamp).unwrap();
    assert_eq!(tz.name, "America/Los_Angeles");
    assert_eq!(tz.offset_seconds, -28800); // UTC-8
}

#[test]
#[should_panic = "Wrong tile"]
fn wrong_tile_edgeinfo() {
    let reader = GraphReader::new(&Config::from_tile_extract(utrecht_tiles("tiles.tar")).unwrap())
        .expect("Failed to create GraphReader");

    let tiles = reader.tiles();
    assert!(tiles.len() >= 2, "This test requires at least two tiles");
    let t1 = reader.graph_tile(tiles[0]).unwrap();
    let t2 = reader.graph_tile(tiles[1]).unwrap();

    let de = t2.directededge(0).unwrap();
    let _ = t1.edgeinfo(de); // should panic
}

#[test]
#[should_panic = "Wrong tile"]
fn wrong_tile_live_traffic() {
    let reader = GraphReader::new(&Config::from_tile_extract(utrecht_tiles("tiles.tar")).unwrap())
        .expect("Failed to create GraphReader");

    let tiles = reader.tiles();
    assert!(tiles.len() >= 2, "This test requires at least two tiles");
    let t1 = reader.graph_tile(tiles[0]).unwrap();
    let t2 = reader.graph_tile(tiles[1]).unwrap();

    let de = t2.directededge(0).unwrap();
    let _ = t1.live_traffic(de); // should panic
}

#[test]
#[should_panic = "Wrong tile"]
fn wrong_tile_node_latlon() {
    let reader = GraphReader::new(&Config::from_tile_extract(utrecht_tiles("tiles.tar")).unwrap())
        .expect("Failed to create GraphReader");

    let tiles = reader.tiles();
    assert!(tiles.len() >= 2, "This test requires at least two tiles");
    let t1 = reader.graph_tile(tiles[0]).unwrap();
    let t2 = reader.graph_tile(tiles[1]).unwrap();

    let node = t2.node(0).unwrap();
    let _ = t1.node_latlon(node); // should panic
}

#[test]
#[should_panic = "Wrong tile"]
fn wrong_tile_node_edges() {
    let reader = GraphReader::new(&Config::from_tile_extract(utrecht_tiles("tiles.tar")).unwrap())
        .expect("Failed to create GraphReader");

    let tiles = reader.tiles();
    assert!(tiles.len() >= 2, "This test requires at least two tiles");
    let t1 = reader.graph_tile(tiles[0]).unwrap();
    let t2 = reader.graph_tile(tiles[1]).unwrap();

    let node = t2.node(0).unwrap();
    let _ = t1.node_edges(node); // should panic
}

#[test]
#[should_panic = "Wrong tile"]
fn wrong_tile_node_transitions() {
    let reader = GraphReader::new(&Config::from_tile_extract(utrecht_tiles("tiles.tar")).unwrap())
        .expect("Failed to create GraphReader");

    let tiles = reader.tiles();
    assert!(tiles.len() >= 2, "This test requires at least two tiles");
    let t1 = reader.graph_tile(tiles[0]).unwrap();
    let t2 = reader.graph_tile(tiles[1]).unwrap();

    let node = t2.node(0).unwrap();
    let _ = t1.node_transitions(node); // should panic
}

#[test]
fn edge_info_shape() {
    let reader =
        GraphReader::new(&Config::from_tile_extract(utrecht_tiles("tiles.tar")).unwrap()).unwrap();

    for tile_id in reader.tiles() {
        let tile = reader.graph_tile(tile_id).unwrap();
        for de in tile.directededges() {
            let ei = tile.edgeinfo(de);

            let shape = ei.shape().collect::<Vec<_>>();
            assert!(shape.len() >= 2, "way {} has {shape:?}", ei.way_id);

            let iter = ei.shape();
            let (lo, hi) = ei.shape().size_hint();
            assert!(lo <= shape.len() && shape.len() <= hi.unwrap());
            assert_eq!(iter.len(), shape.len());
            assert_eq!(iter.is_empty(), shape.is_empty());
            assert_eq!(iter.count(), shape.len()); // `count()` is consuming, check it last

            // Utrecht sits in a tight box, so a bad decode shows up immediately.
            for point in &shape {
                assert!(
                    (UTRECHT_BBOX.0.0..UTRECHT_BBOX.1.0).contains(&point.0),
                    "{point:?}"
                );
                assert!(
                    (UTRECHT_BBOX.0.1..UTRECHT_BBOX.1.1).contains(&point.1),
                    "{point:?}"
                );
            }

            // The shape spans the edge, so it ends at the edge's end node.
            if de.endnode().tile() == tile.id() {
                let end = tile.node_latlon(tile.node(de.endnode().id()).unwrap());
                let shape_end = if de.forward() {
                    shape.last()
                } else {
                    shape.first()
                }
                .unwrap();
                assert!(
                    (shape_end.0 - end.0).abs() < 1e-6,
                    "{shape_end:?} vs {end:?}"
                );
                assert!(
                    (shape_end.1 - end.1).abs() < 1e-6,
                    "{shape_end:?} vs {end:?}"
                );
            }
        }
    }
}
