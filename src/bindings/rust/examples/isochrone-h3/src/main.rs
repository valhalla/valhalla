//! How far can you get from here, drawn as H3 hexagons.
//!
//! Demonstrates:
//!   - hierarchy limits: dropping the local road level once far from the origin
//!   - queue cost, `path_length_m` and remaining budget as three distinct per-node quantities
//!   - one hash map as both tile cache and visited set
//!
//! Labels nodes rather than edges, so turn restrictions are not honoured.
//!
//! Run from the `examples/` workspace, then paste the output into <https://geojson.io>:
//!   cargo run -p isochrone-h3 -- --tiles ../../../../build/test/data/utrecht_tiles/tiles.tar 52.09068,5.12136 --budget 10min
//!   cargo run -p isochrone-h3 -- --tiles ../../../../build/test/data/utrecht_tiles/tiles.tar 52.09068,5.12136 --budget 5km --resolution 9

use std::collections::hash_map::Entry;
use std::path::PathBuf;

use anyhow::{Context, Result, anyhow, bail};
use clap::Parser;
use h3o::{CellIndex, LatLng, Resolution};
use rustc_hash::FxHashMap;
use serde::{Serialize, Serializer, ser::SerializeSeq};
use valhalla::{Access, Actor, DirectedEdge, GraphId, GraphReader, GraphTile, LatLon, TileId};

use crate::{bitset::BitSet, priority_queue::PriorityQueue};

mod bitset;
mod locate;
mod priority_queue;

/// How far the search may wander on each hierarchy level, in metres from the origin.
/// Indexed by [`valhalla::GraphId::level`], see [`valhalla::level`]. Buys speed with coverage.
const HIERARCHY_LIMITS: [u32; 3] = [u32::MAX, 100_000, 10_000];

/// An over-budget node can be followed by an under-budget one, so give up only after this many
/// consecutive over-budget settles.
const OVER_BUDGET_RUN: u32 = 100;

#[derive(Parser)]
#[command(about = "Draw the reachable area from a coordinate as H3 hexagons")]
struct Cli {
    /// Origin, as `lat,lon`.
    #[arg(value_parser = parse_lat_lon)]
    coordinate: LatLon,
    /// How far to go: a time (`30s`, `10min`, `1h`) or a distance (`5000m`, `15km`).
    #[arg(long, default_value = "10min")]
    budget: Budget,
    /// H3 resolution, 0 (coarsest) to 15 (finest). 8 is roughly 0.5 km across.
    #[arg(long, default_value_t = 8)]
    resolution: u8,
    /// Path to the Valhalla tiles.tar.
    #[arg(long)]
    tiles: PathBuf,
    /// How far `/locate` may look for a road to snap onto, in metres.
    #[arg(long, default_value_t = 100)]
    locate_radius: u32,
    /// Refuse to use toll roads. Hand-rolled, as Valhalla's `exclude_tolls` only penalises them.
    #[arg(long)]
    avoid_tolls: bool,
}

/// How far the expansion may go. Time and distance differ only in the per-edge cost.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Budget {
    /// Seconds.
    Time(u32),
    /// Metres.
    Distance(u32),
}

impl Budget {
    fn amount(self) -> u32 {
        match self {
            Budget::Time(v) | Budget::Distance(v) => v,
        }
    }

    fn unit(self) -> &'static str {
        match self {
            Budget::Time(_) => "seconds",
            Budget::Distance(_) => "metres",
        }
    }

    /// What traversing this edge costs against the budget.
    fn edge_cost(self, edge: &DirectedEdge) -> u32 {
        match self {
            Budget::Time(_) => edge_duration_s(edge),
            Budget::Distance(_) => edge.length(),
        }
    }
}

impl std::str::FromStr for Budget {
    type Err = anyhow::Error;

    /// `s` / `min` / `h` are times; `m` / `km` are distances. `m` is metres, never minutes.
    fn from_str(s: &str) -> Result<Self> {
        let split = s
            .find(|c: char| !c.is_ascii_digit() && c != '.')
            .unwrap_or(s.len());
        let (value, unit) = s.split_at(split);
        let value: f64 = value
            .parse()
            .with_context(|| format!("`{s}` does not start with a number"))?;
        if value <= 0.0 {
            bail!("budget must be positive, got `{s}`");
        }

        Ok(match unit {
            "s" => Budget::Time(value.round() as u32),
            "min" => Budget::Time((value * 60.0).round() as u32),
            "h" => Budget::Time((value * 3600.0).round() as u32),
            "m" => Budget::Distance(value.round() as u32),
            "km" => Budget::Distance((value * 1000.0).round() as u32),
            "" => bail!("`{s}` needs a unit: s, min, h, m or km"),
            other => bail!("unknown unit `{other}` in `{s}`; use s, min, h, m or km"),
        })
    }
}

/// Edge traversal time in seconds. `constrained_flow_speed` is 0 on tilesets built without
/// traffic, where the tagged speed is used instead.
fn edge_duration_s(edge: &DirectedEdge) -> u32 {
    let kmh = match edge.constrained_flow_speed() {
        0 => edge.speed(),
        constrained => constrained,
    };
    if kmh == 0 {
        return u32::MAX; // avoids a divide by zero
    }
    (edge.length() as f32 / (kmh as f32 * (1000.0 / 3600.0))).round() as u32
}

fn parse_lat_lon(s: &str) -> Result<LatLon> {
    let (lat, lon) = s
        .split_once(',')
        .ok_or_else(|| anyhow!("expected `lat,lon`, got `{s}`"))?;
    Ok(LatLon(
        lat.trim().parse().context("bad latitude")?,
        lon.trim().parse().context("bad longitude")?,
    ))
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let resolution = Resolution::try_from(cli.resolution)
        .map_err(|e| anyhow!("bad H3 resolution {}: {e}", cli.resolution))?;

    let config = valhalla::ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: cli.tiles.display().to_string(),
            ..Default::default()
        },
        // Valhalla logs to stdout by default, which would land in the middle of the GeoJSON.
        logging: valhalla::config::Logging {
            r#type: "std_err".to_string(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();
    let mut actor = Actor::new(&config).map_err(|e| anyhow!("failed to create Actor: {e}"))?;
    let reader =
        GraphReader::new(&config).map_err(|e| anyhow!("failed to open GraphReader: {e}"))?;

    let origins = locate::locate(&mut actor, cli.coordinate, cli.locate_radius)?;
    let (cells, gates) = expand(&reader, &origins, &cli, resolution, HIERARCHY_LIMITS);
    // To stderr, so it does not land in the GeoJSON.
    eprintln!(
        "hierarchy limits turned away {} nodes and {} level transitions",
        gates.expansion, gates.transition
    );

    println!(
        "{}",
        serde_json::to_string_pretty(&to_geojson(cells, cli.budget))?
    );
    Ok(())
}

/// One node in the frontier. `path_length_m` is always metres, as `HIERARCHY_LIMITS` is a
/// distance, while `remaining_budget` is whatever `--budget` measures.
struct Label {
    id: GraphId,
    path_length_m: u32,
    remaining_budget: u32,
}

/// Expands outward from the snapped edges, folding every settled node into an H3 cell.
/// Ordered by duration, so fast roads are reached first.
fn expand(
    reader: &GraphReader,
    origins: &[GraphId],
    cli: &Cli,
    resolution: Resolution,
    limits: [u32; 3],
) -> (FxHashMap<CellIndex, u32>, GateStats) {
    let mut gates = GateStats::default();
    let allowed = |edge: &DirectedEdge| {
        edge.forwardaccess().intersects(Access::AUTO)
            // Shortcuts summarise the edges beneath them; following both duplicates work.
            && !edge.is_shortcut()
            && !(cli.avoid_tolls && edge.toll())
    };

    // Tile cache and visited set in one map: the `entry()` handing over the tile also marks the
    // node. `GraphTile` is refcounted, so cloning it to release the borrow is a refcount bump.
    let mut cache = FxHashMap::<TileId, (GraphTile, BitSet)>::default();
    let mut queue = PriorityQueue::<u32, Label>::new();
    let mut cells = FxHashMap::<CellIndex, u32>::default();

    for origin in origins {
        let Some(tile) = reader.graph_tile(*origin) else {
            continue;
        };
        let Some(edge) = tile.directededge(origin.id()) else {
            continue;
        };
        if allowed(edge) {
            queue.push(
                0,
                Label {
                    id: edge.endnode(),
                    path_length_m: 0,
                    remaining_budget: cli.budget.amount(),
                },
            );
        }
    }

    let mut over_budget_run = 0;
    while let Some((path_duration_s, label)) = queue.pop() {
        let tile = match cache.entry(label.id.tile()) {
            Entry::Occupied(entry) => {
                let (tile, visited) = entry.into_mut();
                if !visited.insert(label.id.id() as usize) {
                    continue; // already settled
                }
                tile.clone()
            }
            Entry::Vacant(entry) => {
                let Some(tile) = reader.graph_tile(label.id) else {
                    continue; // incomplete tileset
                };
                let mut visited = BitSet::new(tile.nodes().len());
                visited.insert(label.id.id() as usize);
                entry.insert((tile.clone(), visited));
                tile
            }
        };

        // A distance budget does not follow the queue order, so a node popped later can still have
        // budget left and a bare `break` here would lose it.
        if label.remaining_budget == 0 {
            over_budget_run += 1;
            if over_budget_run >= OVER_BUDGET_RUN {
                break;
            }
            continue;
        }
        over_budget_run = 0;

        let Some(node) = tile.node(label.id.id()) else {
            continue;
        };
        if !node.access().intersects(Access::AUTO) {
            continue;
        }

        // Best budget left anywhere in this cell, along the fastest path rather than the cheapest.
        let ll = tile.node_latlon(node);
        if let Ok(point) = LatLng::new(ll.0, ll.1) {
            let cell = point.to_cell(resolution);
            let best = cells.entry(cell).or_insert(0);
            *best = (*best).max(label.remaining_budget);
        }

        // Transitions cover no ground: same cost and length, gated by the level being entered.
        for transition in tile.node_transitions(node) {
            let target = transition.endnode();
            if label.path_length_m < hierarchy_limit(limits, target) {
                queue.push(
                    path_duration_s,
                    Label {
                        id: target,
                        path_length_m: label.path_length_m,
                        remaining_budget: label.remaining_budget,
                    },
                );
            } else {
                gates.transition += 1;
            }
        }

        // Stop expanding along roads of a level we have outgrown.
        if label.path_length_m > hierarchy_limit(limits, label.id) {
            gates.expansion += 1;
            continue;
        }

        for edge in tile.node_edges(node) {
            if !allowed(edge) {
                continue;
            }
            let next = edge.endnode();
            if cache
                .get(&next.tile())
                .is_some_and(|(_, visited)| visited.contains(next.id() as usize))
            {
                continue;
            }
            queue.push(
                path_duration_s.saturating_add(edge_duration_s(edge)),
                Label {
                    id: next,
                    path_length_m: label.path_length_m + edge.length(),
                    remaining_budget: label
                        .remaining_budget
                        .saturating_sub(cli.budget.edge_cost(edge)),
                },
            );
        }
    }

    (cells, gates)
}

/// How often each hierarchy gate turned a node away.
#[derive(Debug, Default, PartialEq)]
struct GateStats {
    /// Nodes dropped because the search had travelled beyond their level's limit.
    expansion: u32,
    /// Level transitions refused because the target level's limit was already exceeded.
    transition: u32,
}

/// The hierarchy limit that applies to a node, by the level it lives on.
fn hierarchy_limit(limits: [u32; 3], node: GraphId) -> u32 {
    limits.get(node.level() as usize).copied().unwrap_or(0)
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

#[derive(Serialize)]
struct Geometry {
    r#type: &'static str,
    /// GeoJSON polygons are an array of linear rings, each in (lon, lat) order and closed.
    #[serde(serialize_with = "as_closed_ring")]
    coordinates: Vec<(f64, f64)>,
}

#[derive(Serialize)]
struct Properties {
    cell: String,
    /// How much budget was left on arrival — shade the hexes by this.
    remaining_budget: u32,
    budget_unit: &'static str,
}

fn to_geojson(cells: FxHashMap<CellIndex, u32>, budget: Budget) -> FeatureCollection {
    let mut features: Vec<_> = cells
        .into_iter()
        .map(|(cell, remaining_budget)| Feature {
            r#type: "Feature",
            geometry: Geometry {
                r#type: "Polygon",
                coordinates: cell.boundary().iter().map(|v| (v.lat(), v.lng())).collect(),
            },
            properties: Properties {
                cell: cell.to_string(),
                remaining_budget,
                budget_unit: budget.unit(),
            },
        })
        .collect();
    // Stable output makes diffing two runs meaningful.
    features.sort_by(|a, b| a.properties.cell.cmp(&b.properties.cell));

    FeatureCollection {
        r#type: "FeatureCollection",
        features,
    }
}

/// Writes a ring as GeoJSON wants it: `[[lon, lat], ...]`, with the first point repeated at the end.
fn as_closed_ring<S: Serializer>(ring: &[(f64, f64)], serializer: S) -> Result<S::Ok, S::Error> {
    let mut rings = serializer.serialize_seq(Some(1))?;
    rings.serialize_element(&ClosedRing(ring))?;
    rings.end()
}

struct ClosedRing<'a>(&'a [(f64, f64)]);

impl Serialize for ClosedRing<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut seq = serializer.serialize_seq(Some(self.0.len() + 1))?;
        for &(lat, lon) in self.0 {
            seq.serialize_element(&(lon, lat))?;
        }
        if let Some(&(lat, lon)) = self.0.first() {
            seq.serialize_element(&(lon, lat))?;
        }
        seq.end()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;

    #[test]
    fn parses_budget() {
        assert_eq!("30s".parse::<Budget>().unwrap(), Budget::Time(30));
        assert_eq!("10min".parse::<Budget>().unwrap(), Budget::Time(600));
        assert_eq!("1h".parse::<Budget>().unwrap(), Budget::Time(3600));
        assert_eq!("1.5h".parse::<Budget>().unwrap(), Budget::Time(5400));

        assert_eq!("5000m".parse::<Budget>().unwrap(), Budget::Distance(5000));
        assert_eq!("15km".parse::<Budget>().unwrap(), Budget::Distance(15000));
        assert_eq!("2.5km".parse::<Budget>().unwrap(), Budget::Distance(2500));

        // Metres vs minutes
        assert_eq!("15m".parse::<Budget>().unwrap(), Budget::Distance(15));
        assert_eq!("15min".parse::<Budget>().unwrap(), Budget::Time(900));

        // Invalid inputs
        assert!("".parse::<Budget>().is_err());
        assert!("abc".parse::<Budget>().is_err());
        assert!("10 min".parse::<Budget>().is_err());
        assert!("-5min".parse::<Budget>().is_err());
        assert!("0km".parse::<Budget>().is_err());
        assert!("10fortnights".parse::<Budget>().is_err());
    }
}
