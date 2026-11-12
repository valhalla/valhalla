//! Builds a tar extract of Valhalla graph tiles, with an `index.bin` for fast loading and
//! optionally an empty `traffic.tar` to go with it.
//!
//! Exit codes: 0 on success, 1 on any error, 2 on invalid command-line usage.
#![forbid(unsafe_code)]
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented
)]

use std::env;
use std::io::{self, Write};
use std::path::PathBuf;
use std::process::ExitCode;

use clap::{ArgAction, ArgGroup, Parser};
use tracing::Level;
use valhalla_tools::extract::{self, Options, Rect, TileFilter};

/// Replaces the Geofabrik index URL, for tests and mirrors.
const GEOFABRIK_INDEX_URL_VAR: &str = "VALHALLA_GEOFABRIK_INDEX_URL";

#[derive(Parser)]
#[command(
    about = "Builds a tar extract from the tiles in mjolnir.tile_dir to the path specified in mjolnir.tile_extract.",
    group(ArgGroup::new("area").args(["bbox", "geojson_dir", "regions"]))
)]
struct Cli {
    /// Absolute or relative path to the Valhalla config JSON.
    #[arg(short, long)]
    config: Option<PathBuf>,
    /// Inline JSON config; its top-level keys replace those of --config.
    #[arg(short, long)]
    inline_config: Option<String>,
    /// Build the extract from the existing tar at 'mjolnir.tile_extract' and save it to this path.
    #[arg(short, long)]
    extract_tar: Option<PathBuf>,
    /// Overwrite an existing output tar.
    #[arg(short = 'O', long)]
    overwrite: bool,
    /// Also write a traffic.tar skeleton.
    #[arg(short = 't', long)]
    with_traffic: bool,
    /// Only archive the tiles touching this bbox.
    #[arg(
        short,
        long,
        value_name = "MINX,MINY,MAXX,MAXY",
        allow_hyphen_values = true,
        value_parser = extract::parse_bbox
    )]
    bbox: Option<Rect>,
    /// Directory with .geojson files; only archive the tiles touching their polygons.
    #[arg(short, long)]
    geojson_dir: Option<PathBuf>,
    /// Geofabrik region name or id (e.g. 'switzerland', 'europe/france'), repeatable.
    /// Use '--region list' to show all available regions.
    #[arg(short, long = "region", value_name = "REGION")]
    regions: Vec<String>,
    /// Accumulative verbosity: -v for info, -vv for debug.
    #[arg(short, long = "verbosity", action = ArgAction::Count)]
    verbosity: u8,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    init_logging(cli.verbosity);
    match run(cli) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("error: {error:#}");
            ExitCode::FAILURE
        }
    }
}

fn init_logging(verbosity: u8) {
    let level = match verbosity {
        0 => Level::ERROR,
        1 => Level::INFO,
        _ => Level::DEBUG,
    };
    tracing_subscriber::fmt()
        .with_writer(io::stderr)
        .with_max_level(level)
        .init();
}

fn run(cli: Cli) -> anyhow::Result<()> {
    let geofabrik_index_url = env::var(GEOFABRIK_INDEX_URL_VAR)
        .unwrap_or_else(|_| extract::GEOFABRIK_INDEX_URL.to_owned());

    if cli
        .regions
        .iter()
        .any(|region| region.eq_ignore_ascii_case("list"))
    {
        let listing = extract::region_list(&geofabrik_index_url)?;
        io::stdout().lock().write_all(listing.as_bytes())?;
        return Ok(());
    }

    let filter = if let Some(bbox) = cli.bbox {
        TileFilter::Bbox(bbox)
    } else if let Some(dir) = cli.geojson_dir {
        TileFilter::GeojsonDir(dir)
    } else if !cli.regions.is_empty() {
        TileFilter::Regions(cli.regions)
    } else {
        TileFilter::All
    };
    extract::run(&Options {
        config: cli.config,
        inline_config: cli.inline_config,
        extract_tar: cli.extract_tar,
        overwrite: cli.overwrite,
        with_traffic: cli.with_traffic,
        filter,
        geofabrik_index_url,
    })?;
    Ok(())
}
