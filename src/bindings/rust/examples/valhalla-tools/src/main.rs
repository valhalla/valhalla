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
use tracing::{Level, error, warn};
use tracing_subscriber::filter::Targets;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use valhalla_tools::extract::{self, Options, TileFilter, WriteMode};

use log_format::{LineFormat, LogFormat};
use valhalla_logging::Logger;

/// Events of this target are logged whatever the verbosity, as the C++ logger does.
const UNFILTERED: &str = "valhalla_build_extract::unfiltered";

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
    /// Write each tar to a temporary file and rename it over the output path once complete,
    /// instead of rewriting the output in place; an output symlink is replaced, not followed.
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
    let logger = match logger(&cli) {
        Ok(logger) => logger,
        Err(message) => {
            init_logging(level, cli.log_format, Logger::stderr());
            error!("{message}");
            return ExitCode::FAILURE;
        }
    };
    let announce_default = logger.announce_default;
    init_logging(level, cli.log_format, logger);
    if announce_default {
        warn!(
            target: UNFILTERED,
            "No top-level 'logging' section in config, using default logger."
        );
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

/// Only the Valhalla format reads the config's `logging` section.
fn logger(cli: &Cli) -> Result<Logger, String> {
    if cli.log_format == LogFormat::Python {
        return Ok(Logger::stderr());
    }
    match extract::config_section(cli.config.as_deref(), Some(&cli.inline_config), "logging") {
        Ok(section) => valhalla_logging::from_section(section.as_ref()),
        // The run reports the unreadable config, through the logger the C++ would start with.
        Err(_) => Ok(Logger::default_console()),
    }
}

fn init_logging(level: Level, format: LogFormat, logger: Logger) {
    let filter = Targets::new()
        .with_default(level)
        .with_target(UNFILTERED, Level::TRACE);
    tracing_subscriber::fmt()
        .with_writer(logger.target)
        .with_max_level(Level::TRACE)
        .event_format(LineFormat {
            format,
            color: logger.color,
        })
        .finish()
        .with(filter)
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
