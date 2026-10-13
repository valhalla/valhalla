use pretty_assertions::assert_eq;
use std::collections::HashMap;

use valhalla::{Access, Config, GraphId, GraphReader, RoadClass};

mod common;
use common::utrecht_tiles;

#[test]
fn furthest_reachable_node() {
    let reader = GraphReader::new(&Config::from_tile_extract(utrecht_tiles("tiles.tar")).unwrap())
        .expect("Failed to create GraphReader");

    // A stretch of the A27 motorway, OSM way 7007726
    let (tile, edges) = reader
        .tiles()
        .into_iter()
        .find_map(|tile_id| {
            let tile = reader.graph_tile(tile_id).unwrap();
            let motorway_edges = tile
                .directededges()
                .iter()
                .enumerate()
                .filter_map(|(i, de)| (tile.edgeinfo(de).way_id == 7007726).then_some(i as u32))
                .collect::<Vec<_>>();
            (!motorway_edges.is_empty()).then_some((tile, motorway_edges))
        })
        .unwrap();

    let furthest_node_distance = |reader: &GraphReader, access: Access, node_id: GraphId| {
        let mut tile_id = node_id.tile();
        let mut tile = reader.graph_tile(tile_id).unwrap();

        // labels keep the shortest distance to each node we've found so far
        let mut node_labels = HashMap::new();
        node_labels.insert(node_id, 0u32);
        // Poor man's priority queue for expansion. A simple stack in this case as order doesn't matter here
        let mut to_visit = vec![node_id];
        while let Some(node_id) = to_visit.pop() {
            if node_id.tile() != tile_id {
                tile_id = node_id.tile();
                tile = reader.graph_tile(tile_id).unwrap();
            }
            let node = tile.node(node_id.id()).unwrap();
            if !node.access().intersects(access) {
                continue;
            }
            let curr_length = *node_labels.get(&node_id).unwrap();

            for transition in tile.node_transitions(node) {
                let next_node_id = transition.endnode();
                if next_node_id.level() == valhalla::level::LOCAL {
                    continue; // exploring all edges in the 2nd level takes too long
                }

                if node_labels
                    .get(&next_node_id)
                    .is_none_or(|&length| curr_length < length)
                {
                    node_labels.insert(next_node_id, curr_length);
                    to_visit.push(next_node_id);
                }
            }

            for de in tile.node_edges(node) {
                if !de.forwardaccess().intersects(access) {
                    continue;
                }

                let next_node_id = de.endnode();
                let next_length = curr_length + de.length();
                if node_labels
                    .get(&next_node_id)
                    .is_none_or(|&length| next_length < length)
                {
                    node_labels.insert(next_node_id, next_length);
                    to_visit.push(next_node_id);
                }
            }
        }
        node_labels.into_values().max().unwrap()
    };

    assert_eq!(edges.len(), 2);
    // these two nodes are at the ends of the stretch
    let first_edge = tile.directededge(edges[0]).unwrap();
    assert_eq!(first_edge.road_class(), RoadClass::kMotorway);
    let first_node = first_edge.endnode();

    let second_edge = tile.directededge(edges[1]).unwrap();
    assert_eq!(second_edge.road_class(), RoadClass::kMotorway);
    let second_node = second_edge.endnode();

    assert_eq!(first_node.level(), valhalla::level::HIGHWAY);
    assert_eq!(second_node.level(), valhalla::level::HIGHWAY);

    // What Valhalla's `auto` costing accepts.
    let auto = Access::AUTO | Access::HOV;
    assert!(furthest_node_distance(&reader, auto, first_node) > 10_000);
    assert!(furthest_node_distance(&reader, auto, second_node) > 10_000);

    // no luck for bikes and pedestrians on a motorway
    let bike = Access::BICYCLE;
    assert_eq!(furthest_node_distance(&reader, bike, first_node), 0);
    assert_eq!(furthest_node_distance(&reader, bike, second_node), 0);

    let pedestrian = Access::PEDESTRIAN;
    assert_eq!(furthest_node_distance(&reader, pedestrian, first_node), 0);
    assert_eq!(furthest_node_distance(&reader, pedestrian, second_node), 0);
}
