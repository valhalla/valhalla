//! Builds a tar extract of Valhalla graph tiles, with an `index.bin` for fast loading and
//! optionally an empty `traffic.tar` to go with it.
//!
//! Exit codes, as with the Python script: 0 on success, 1 on any error (an invalid `--bbox`
//! included), 2 when the arguments cannot be parsed.
#![forbid(unsafe_code)]
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented
)]

mod log_format;
mod valhalla_logging;

use std::env;
use std::io::{self, Write};
use std::path::PathBuf;
use std::process::ExitCode;

use clap::{ArgAction, ArgGroup, Parser};
use serde_json::{Map, Value};
use tracing::{Level, error, warn};
use valhalla_tools::extract::{self, Options, TileFilter, WriteMode};

use log_format::{LineFormat, LogFormat};
use valhalla_logging::{LogTarget, Logger};

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
    /// Inline JSON config, will override --config JSON if present.
    #[arg(short, long, default_value = "{}")]
    inline_config: String,
    /// If specified, will build an extract from an existing tar file at 'mjolnir.tile_extract'
    /// and save it to this specified path.
    #[arg(short, long)]
    extract_tar: Option<PathBuf>,
    /// Overwrites an output tar file.
    #[arg(short = 'O', long)]
    overwrite: bool,
    /// Write each tar to a temporary file and rename it over the output once complete, instead
    /// of rewriting the output in place.
    #[arg(long)]
    atomic: bool,
    /// Flag to add a traffic.tar skeleton.
    #[arg(short = 't', long)]
    with_traffic: bool,
    /// If specified, will only archive tiles which are intersecting this bbox. In
    /// 'minx,miny,maxx,maxy' format.
    #[arg(short, long, allow_hyphen_values = true)]
    bbox: Option<String>,
    /// Absolute or relative path to directory with .geojson files used as input to tile
    /// intersection.
    #[arg(short, long)]
    geojson_dir: Option<PathBuf>,
    /// Geofabrik region name or ID (e.g. 'switzerland', 'europe/france'). Pass multiple times to
    /// combine regions. Use '--region list' to show all available regions.
    #[arg(short, long = "region", value_name = "REGION")]
    regions: Vec<String>,
    /// Accumulative verbosity flags; -v: INFO, -vv: DEBUG.
    #[arg(short, long = "verbosity", action = ArgAction::Count)]
    verbosity: u8,
    /// Log line format: the Python script's, on stderr, or Valhalla's C++ logger's, which also
    /// follows the config's top-level 'logging' section.
    #[arg(long, value_enum, default_value_t = LogFormat::Python)]
    log_format: LogFormat,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    // Listing regions logs at info level even without -v, as the script does.
    let verbosity = if lists_regions(&cli) {
        cli.verbosity.max(1)
    } else {
        cli.verbosity
    };
    let level = match verbosity {
        0 => Level::ERROR,
        1 => Level::INFO,
        _ => Level::DEBUG,
    };
    match logger(&cli) {
        Ok((logger, missing_section)) => {
            init_logging(level, cli.log_format, logger);
            if missing_section {
                warn!("No top-level 'logging' section in config, using default logger.");
            }
        }
        Err(message) => {
            let stderr = Logger {
                target: LogTarget::Stderr,
                color: false,
            };
            init_logging(level, cli.log_format, stderr);
            error!("{message}");
            return ExitCode::FAILURE;
        }
    }
    match run(cli) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            error!("{error:#}");
            ExitCode::FAILURE
        }
    }
}

fn lists_regions(cli: &Cli) -> bool {
    cli.regions
        .iter()
        .any(|region| region.eq_ignore_ascii_case("list"))
}

/// The logger and whether the config lacks a `logging` section, which only the Valhalla format reads.
fn logger(cli: &Cli) -> Result<(Logger, bool), String> {
    if cli.log_format == LogFormat::Python {
        let stderr = Logger {
            target: LogTarget::Stderr,
            color: false,
        };
        return Ok((stderr, false));
    }
    let section =
        extract::config_section(cli.config.as_deref(), Some(&cli.inline_config), "logging");
    match section {
        Some(Value::Object(section)) => {
            Ok((valhalla_logging::from_section(Some(&section))?, false))
        }
        // A scalar section has no keys, so no logger type either.
        Some(_) => Ok((valhalla_logging::from_section(Some(&Map::new()))?, false)),
        None => Ok((valhalla_logging::from_section(None)?, true)),
    }
}

fn init_logging(level: Level, format: LogFormat, logger: Logger) {
    tracing_subscriber::fmt()
        .with_writer(logger.target)
        .with_max_level(level)
        .event_format(LineFormat {
            format,
            color: logger.color,
        })
        .init();
}

fn run(cli: Cli) -> anyhow::Result<()> {
    let geofabrik_index_url = env::var(GEOFABRIK_INDEX_URL_VAR)
        .unwrap_or_else(|_| extract::GEOFABRIK_INDEX_URL.to_owned());

    if lists_regions(&cli) {
        let listing = extract::region_list(&geofabrik_index_url)?;
        io::stdout().lock().write_all(listing.as_bytes())?;
        return Ok(());
    }

    // Empty values count as not given, as in the script.
    let extract_tar = cli.extract_tar.filter(|path| !path.as_os_str().is_empty());
    let filter = if let Some(bbox) = cli.bbox.filter(|bbox| !bbox.is_empty()) {
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
        inline_config: Some(cli.inline_config),
        extract_tar,
        overwrite: cli.overwrite,
        write_mode: if cli.atomic {
            WriteMode::Atomic
        } else {
            WriteMode::InPlace
        },
        with_traffic: cli.with_traffic,
        filter,
        geofabrik_index_url,
    })?;
    Ok(())
}
