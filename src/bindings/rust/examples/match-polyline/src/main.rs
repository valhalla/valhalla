//! Map-matches a polyline with `Actor::trace_attributes`: typed `proto::Options` in, `proto::Api` out.
//!
//! Run from the `examples/` workspace:
//!   cargo run -p match-polyline -- \
//!     --tiles ../../../../build/test/data/utrecht_tiles/tiles.tar \
//!     --polyline6 'quijbBqpnwHfJxc@bBdJrDfSdAzFX|AHd@bG~[|AnIdArGbAo@z@m@`EuClO}MjE}E~NkPaAuC'

use std::path::PathBuf;

use anyhow::{Result, anyhow, bail};
use clap::Parser;
use valhalla::{
    Actor, ConfigBuilder, GraphId,
    proto::{
        self, RoadClass,
        options::{Format, HasEncodedPolyline::EncodedPolyline, HasSearchRadius::SearchRadius},
        trip_leg::Use,
    },
};

/// Valhalla's default of 50m is loose enough to snap onto parallel roads.
const SEARCH_RADIUS_M: f32 = 20.0;

#[derive(Parser)]
#[command(about = "Map-match a polyline and print the matched edges")]
struct Cli {
    /// Polyline with 6 decimal places of precision.
    #[arg(long)]
    polyline6: String,
    /// Path to the Valhalla tiles.tar.
    #[arg(long)]
    tiles: PathBuf,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    let config = ConfigBuilder {
        mjolnir: valhalla::config::Mjolnir {
            tile_extract: cli.tiles.display().to_string(),
            ..Default::default()
        },
        // Valhalla logs to stdout by default, which would mix with the output
        logging: valhalla::config::Logging {
            r#type: "std_err".to_string(),
            ..Default::default()
        },
        ..Default::default()
    }
    .build();

    let mut actor = Actor::new(&config).map_err(|e| anyhow!("failed to create Actor: {e}"))?;

    let request = proto::Options {
        costing_type: proto::costing::Type::Auto as i32,
        format: Format::Pbf as i32,
        shape_match: proto::ShapeMatch::MapSnap as i32,
        has_encoded_polyline: Some(EncodedPolyline(cli.polyline6)),
        has_search_radius: Some(SearchRadius(SEARCH_RADIUS_M)),
        ..Default::default()
    };

    let response = actor.trace_attributes(&request);
    let Ok(valhalla::Response::Pbf(api)) = response else {
        bail!("expected a protobuf response, got {response:?}");
    };

    // `trip` and `edge` are optional, the rest are repeated fields — walk, do not index.
    let edges: Vec<_> = api
        .trip
        .iter()
        .flat_map(|trip| &trip.routes)
        .flat_map(|route| &route.legs)
        .flat_map(|leg| &leg.node)
        .filter_map(|node| node.edge.as_ref())
        .collect();
    if edges.is_empty() {
        bail!("nothing matched, is the polyline inside the tileset?");
    }

    println!("matched {} edges", edges.len());
    println!(
        "{:<22} {:>11}  {:<24} {:<14} {:<12} {:>8}  along edge",
        "graph_id", "way_id", "name", "use", "road_class", "length"
    );
    for edge in edges {
        // `name` is a repeated `StreetName`, not a string.
        let name = edge.name.first().map_or("", |n| n.value.as_str());
        println!(
            "{:<22} {:>11}  {:<24} {:<14} {:<12} {:>7.0}m  {:.2}..{:.2}",
            GraphId::new(edge.id).to_string(),
            edge.way_id,
            name,
            format!("{:?}", Use::try_from(edge.r#use).unwrap_or(Use::KRoadUse)),
            format!(
                "{:?}",
                RoadClass::try_from(edge.road_class).unwrap_or_default()
            ),
            edge.length_km * 1000.0,
            edge.source_along_edge,
            edge.target_along_edge,
        );
    }
    Ok(())
}
