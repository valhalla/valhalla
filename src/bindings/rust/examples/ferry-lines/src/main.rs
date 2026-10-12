//! Extracts every ferry route from a Valhalla tileset and prints it as GeoJSON.
//!
//! Demonstrates:
//!   - scanning a whole tileset in parallel with one shared `GraphReader`
//!   - recovering an edge's *begin* node via `opp_index`
//!   - following a route across tile boundaries and several edges of one OSM way
//!   - country, state and timezone straight from the tiles, no geocoder
//!
//! Needs a coastal tileset; the repo's Andorra fixture is landlocked. From `examples/`:
//!   cargo run -p ferry-lines -- --tiles path/to/denmark.tar

use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Result, anyhow};
use clap::Parser;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use serde::{Serialize, Serializer, ser::SerializeSeq};
use valhalla::{
    Access, DirectedEdge, EdgeUse, GraphId, GraphReader, GraphTile, LatLon, NodeInfo, TimeZoneInfo,
};

/// Ferries mapped as a loop can never reach land again; give up rather than spin.
const MAX_EDGES_PER_FERRY: u32 = 50;

#[derive(Parser)]
#[command(about = "Extract ferry routes from a Valhalla tileset as GeoJSON")]
struct Cli {
    /// Path to the Valhalla tiles.tar.
    #[arg(long)]
    tiles: PathBuf,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let config = valhalla::ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: cli.tiles.display().to_string(),
            ..Default::default()
        },
        // Valhalla logs to stdout by default; stdout here is GeoJSON.
        logging: valhalla::config::Logging {
            r#type: "std_err".to_string(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    let reader = GraphReader::new(&config).map_err(|e| anyhow!("failed to open tiles: {e}"))?;
    let now = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs();

    let features = ferry_routes(&reader)
        .into_iter()
        .map(|route| Feature::new(&reader, route, now))
        .collect();

    println!(
        "{}",
        serde_json::to_string_pretty(&FeatureCollection {
            r#type: "FeatureCollection",
            features,
        })?
    );
    Ok(())
}

/// A ferry route: one OSM way's worth of ferry edges, from land to land.
struct FerryRoute {
    /// OSM way the ferry was mapped as. Not unique — one way can yield several routes.
    way_id: u64,
    /// Full geometry in (lat, lon) order.
    geometry: Box<[LatLon]>,
    /// Total length in metres.
    length_m: u32,
    /// Graph nodes where the route starts and ends. Both are on land.
    from_node: GraphId,
    to_node: GraphId,
}

fn is_ferry_edge(de: &DirectedEdge) -> bool {
    matches!(de.use_type(), EdgeUse::kFerry | EdgeUse::kRailFerry)
}

/// Recovers the node an edge *starts* from: an edge stores only `endnode()`, so follow the
/// opposing edge at `opp_index()` back. `end_tile` must hold `edge.endnode()`.
fn begin_node(end_tile: &GraphTile, edge: &DirectedEdge) -> Option<GraphId> {
    let end_node = end_tile.node(edge.endnode().id())?;
    let opposing = end_tile
        .node_edges(end_node)
        .get(edge.opp_index() as usize)?;
    Some(opposing.endnode())
}

/// Extracts every ferry route in the tileset.
fn ferry_routes(reader: &GraphReader) -> Vec<FerryRoute> {
    reader
        .tiles()
        // Reading tiles is IO bound and `GraphReader` is `Sync`, so threads share one by reference.
        .into_par_iter()
        .flat_map(|tile_id| {
            let Some(tile) = reader.graph_tile(tile_id) else {
                return Vec::new();
            };
            tile.directededges()
                .iter()
                .filter(|e| is_ferry_edge(e))
                // Shortcuts carry no OSM way (`way_id == 0`).
                .filter(|e| !e.is_shortcut())
                .filter(|e| e.forwardaccess().intersects(Access::AUTO))
                .filter_map(|edge| resolve_ferry_route(reader, &tile, edge))
                .collect::<Vec<_>>()
        })
        .collect()
}

/// Walks one ferry route from the given edge until it reaches land, or gives up. `None` unless the
/// edge is the *first* one leaving land, so each route is emitted once.
fn resolve_ferry_route(
    reader: &GraphReader,
    begin_tile: &GraphTile,
    begin_edge: &DirectedEdge,
) -> Option<FerryRoute> {
    let end_node_id = begin_edge.endnode();
    let mut end_tile = if end_node_id.tile() == begin_tile.id() {
        begin_tile.clone() // `GraphTile` is refcounted, so this is cheap.
    } else {
        reader.graph_tile(end_node_id)?
    };

    // An edge is stored in the tile of the node it starts from, so this lands in `begin_tile`.
    let begin_node_id = begin_node(&end_tile, begin_edge)?;
    debug_assert_eq!(begin_node_id.tile(), begin_tile.id());
    let begin_node = begin_tile.node(begin_node_id.id())?;

    // Only start from a landing: the first edge must be reachable *from* land by car.
    if !any_node_edge(reader, begin_tile, begin_node, |de| {
        !is_ferry_edge(de) && de.reverseaccess().intersects(Access::AUTO)
    }) {
        return None;
    }

    let ei = begin_tile.edgeinfo(begin_edge);
    let way_id = ei.way_id;

    // Shape is stored once per OSM way, so the reverse edge of a pair walks it backwards.
    let mut geometry = ei.shape().collect::<Vec<_>>();
    if !begin_edge.forward() {
        geometry.reverse();
    }
    let mut length_m = begin_edge.length();

    // Follow the remaining edges of the same OSM way until we hit land again.
    let mut opp_index = begin_edge.opp_index() as usize; // the way back, to be skipped
    let mut end_node_id = end_node_id;
    for _ in 0..MAX_EDGES_PER_FERRY {
        let end_node = end_tile.node(end_node_id.id())?;

        // Check for land *before* another ferry edge: some ways loop through several landings.
        if any_node_edge(reader, &end_tile, end_node, |de| {
            !is_ferry_edge(de) && de.forwardaccess().intersects(Access::AUTO)
        }) {
            return Some(FerryRoute {
                way_id,
                geometry: geometry.into(),
                length_m,
                from_node: begin_node_id,
                to_node: end_node_id,
            });
        }

        // Continue along the same OSM way; a ferry connecting only to other ferries stops here.
        let (next_edge, shape) = end_tile
            .node_edges(end_node)
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != opp_index)
            .find_map(|(_, de)| {
                let ei = end_tile.edgeinfo(de);
                (ei.way_id == way_id).then_some((de, ei.shape()))
            })?;

        // Drop the point shared with the previous edge. A forward edge is already in edge order;
        // a reverse one is appended and flipped in place.
        if next_edge.forward() {
            geometry.extend(shape.skip(1));
        } else {
            let start = geometry.len();
            geometry.extend(shape);
            geometry.pop(); // last in storage order is the edge's first
            geometry[start..].reverse();
        }
        length_m += next_edge.length();

        end_node_id = next_edge.endnode();
        opp_index = next_edge.opp_index() as usize;
        if end_node_id.tile() != end_tile.id() {
            end_tile = reader.graph_tile(end_node_id)?;
        }
    }

    eprintln!("warning: gave up following OSM way {way_id} after {MAX_EDGES_PER_FERRY} edges");
    None
}

/// Where a ferry landing is, from the node's admin index and timezone id — no geocoder.
#[derive(Default)]
struct Place {
    country_iso: Option<String>,
    state_iso: Option<String>,
    timezone: Option<String>,
}

impl Place {
    fn lookup(reader: &GraphReader, node_id: GraphId, now: u64) -> Self {
        let Some(tile) = reader.graph_tile(node_id) else {
            return Self::default();
        };
        let Some(node) = tile.node(node_id.id()) else {
            return Self::default();
        };

        let admin = tile.admin_info(node.admin_index());
        // `admin_info` leaves these empty rather than absent.
        let non_empty = |s: &str| (!s.is_empty()).then(|| s.to_owned());
        Self {
            country_iso: admin.as_ref().and_then(|a| non_empty(&a.country_iso)),
            state_iso: admin.as_ref().and_then(|a| non_empty(&a.state_iso)),
            timezone: TimeZoneInfo::from_id(node.timezone(), now).map(|tz| tz.name),
        }
    }
}

#[derive(Serialize)]
struct FeatureCollection {
    r#type: &'static str,
    features: Vec<Feature>,
}

#[derive(Serialize)]
struct Feature {
    r#type: &'static str,
    geometry: Geometry,
    properties: Properties,
}

impl Feature {
    fn new(reader: &GraphReader, route: FerryRoute, now: u64) -> Self {
        let from = Place::lookup(reader, route.from_node, now);
        let to = Place::lookup(reader, route.to_node, now);
        Self {
            r#type: "Feature",
            geometry: Geometry {
                r#type: "LineString",
                coordinates: route.geometry,
            },
            properties: Properties {
                way_id: route.way_id,
                length_m: route.length_m,
                from_country_iso: from.country_iso,
                from_state_iso: from.state_iso,
                to_country_iso: to.country_iso,
                to_state_iso: to.state_iso,
                timezone: from.timezone,
            },
        }
    }
}

#[derive(Serialize)]
struct Geometry {
    r#type: &'static str,
    /// GeoJSON is (lon, lat); everything else here is (lat, lon).
    #[serde(serialize_with = "as_lon_lat")]
    coordinates: Box<[LatLon]>,
}

#[derive(Serialize)]
struct Properties {
    way_id: u64,
    length_m: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    from_country_iso: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    from_state_iso: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    to_country_iso: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    to_state_iso: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    timezone: Option<String>,
}

fn as_lon_lat<S: Serializer>(geometry: &[LatLon], serializer: S) -> Result<S::Ok, S::Error> {
    let mut seq = serializer.serialize_seq(Some(geometry.len()))?;
    for &LatLon(lat, lon) in geometry {
        seq.serialize_element(&(lon, lat))?;
    }
    seq.end()
}

/// Whether any edge at this node, including across node transitions, satisfies `predicate`.
fn any_node_edge(
    reader: &GraphReader,
    tile: &GraphTile,
    node: &NodeInfo,
    predicate: impl Fn(&DirectedEdge) -> bool,
) -> bool {
    tile.node_edges(node).iter().any(&predicate)
        || tile.node_transitions(node).iter().any(|t| {
            let node_id = t.endnode();
            reader
                .graph_tile(node_id)
                .and_then(|tile| {
                    let node = tile.node(node_id.id())?;
                    Some(tile.node_edges(node).iter().any(&predicate))
                })
                .unwrap_or(false)
        })
}
