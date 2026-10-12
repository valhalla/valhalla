//! Is this address stuck behind a toll, and how far is the nearest highway?
//!
//! Demonstrates:
//!   - `Actor::locate` snaps a coordinate the way the routing engine does
//!   - `Exhausted` is an answer: the reachable subgraph ran out
//!   - labels nodes, not edges, so turn restrictions are not honoured
//!
//! Run from the `examples/` workspace, against the repo's Andorra tileset:
//!   cargo run -p reachability -- --tiles ../tests/andorra/tiles.tar 42.46372,1.49129
//!       # Sant Julià de Lòria: nearest highway 362m away
//!   cargo run -p reachability -- --tiles ../tests/andorra/tiles.tar 42.5443,1.7285
//!       # Pas de la Casa, on Túnel d'Envalira: toll_zone

use std::path::PathBuf;

use anyhow::{Context, Result, anyhow};
use clap::Parser;
use serde::Serialize;
use valhalla::{Access, Actor, DirectedEdge, GraphReader, LatLon, NodeInfo, RoadClass};

use crate::dijkstra::{CachedGraphReader, SearchResult};

mod bitset;
mod dijkstra;
mod locate;
mod priority_queue;

/// Toll-free distance *along the network* required to not count as a toll island.
const TOLL_ZONE_REACHABILITY: u32 = 1500;
/// How far to look for a highway before giving up.
const HIGHWAY_SEARCH_RADIUS: u32 = 10_000;

#[derive(Parser)]
#[command(about = "Toll-island and nearest-highway analysis over the Valhalla road graph")]
struct Cli {
    /// Coordinate to analyse, as `lat,lon`.
    #[arg(value_parser = parse_lat_lon)]
    coordinate: LatLon,
    /// Path to the Valhalla tiles.tar.
    #[arg(long)]
    tiles: PathBuf,
    /// How far `/locate` may look for a road to snap onto, in metres.
    #[arg(long, default_value_t = 100)]
    locate_radius: u32,
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

/// What the analysis concluded about a coordinate.
#[derive(Debug, Serialize, PartialEq)]
struct Verdict {
    /// You cannot get `TOLL_ZONE_REACHABILITY` metres away without paying.
    toll_zone: bool,
    /// Distance to the nearest highway in metres, if one is within `HIGHWAY_SEARCH_RADIUS`.
    nearest_highway_distance: Option<u32>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let config = valhalla::ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: cli.tiles.display().to_string(),
            ..Default::default()
        },
        // Valhalla logs to stdout by default, which would corrupt this example's own output.
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

    let verdict = analyse(&mut actor, reader, cli.coordinate, cli.locate_radius)?;
    println!("{}", serde_json::to_string_pretty(&verdict)?);
    Ok(())
}

fn analyse(
    actor: &mut Actor,
    reader: GraphReader,
    coordinate: LatLon,
    locate_radius: u32,
) -> Result<Verdict> {
    // Snap like the router does; the search starts at the far end of the snapped edge.
    let edges = locate::locate(actor, coordinate, locate_radius)?;

    // For this analytics we are not interested in private areas
    let node_filter =
        |node: &NodeInfo| node.access().intersects(Access::AUTO) && !node.private_access();
    let edge_filter = |edge: &DirectedEdge| edge.forwardaccess().intersects(Access::AUTO);
    // Hand-rolled, as Valhalla's `exclude_tolls` only penalises tolls.
    let toll_free = |edge: &DirectedEdge| edge_filter(edge) && !edge.toll();
    // `RoadClass` is ordered by importance: below primary is motorway and trunk.
    let is_highway = |edge: &DirectedEdge| edge.road_class() < RoadClass::kPrimary;

    // One cache for all three searches.
    let mut cache = CachedGraphReader::new(reader);

    let nearest_highway_distance = match dijkstra::search(
        &mut cache,
        &edges,
        node_filter,
        edge_filter,
        is_highway,
        HIGHWAY_SEARCH_RADIUS,
    ) {
        SearchResult::Found(distance) => Some(distance),
        SearchResult::Exhausted | SearchResult::MaxDistanceReached => None,
    };

    // First: is there a toll edge nearby at all?
    let toll_probe = dijkstra::search(
        &mut cache,
        &edges,
        node_filter,
        edge_filter,
        |edge| edge.toll(),
        TOLL_ZONE_REACHABILITY,
    );
    // Then: on toll-free edges only, can we still get far enough away?
    let toll_free_reach = dijkstra::search(
        &mut cache,
        &edges,
        node_filter,
        toll_free,
        |_| false,
        TOLL_ZONE_REACHABILITY,
    );

    // A toll road in reach, *and* the toll-free graph runs out before you get far enough away.
    // `Exhausted` also covers starting on the toll road itself, or running off the tileset edge.
    let toll_zone =
        matches!(toll_probe, SearchResult::Found(_)) && toll_free_reach == SearchResult::Exhausted;

    Ok(Verdict {
        toll_zone,
        nearest_highway_distance,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;

    #[test]
    fn parses_coordinates() {
        assert_eq!(parse_lat_lon("42.5,1.5").unwrap(), LatLon(42.5, 1.5));
        assert_eq!(parse_lat_lon(" 42.5 , 1.5 ").unwrap(), LatLon(42.5, 1.5));
        assert!(parse_lat_lon("42.5").is_err());
        assert!(parse_lat_lon("north,east").is_err());
    }
}
