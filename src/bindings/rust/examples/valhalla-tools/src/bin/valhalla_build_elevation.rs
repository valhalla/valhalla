//! Downloads the elevation tiles that intersect the polygons of GeoJSON files, a bounding box or
//! the tiles of a Valhalla graph, from the public AWS Terrain Tiles bucket.
//!
//! Exit codes: 0 when every tile was downloaded or already present, 1 on any error, including
//! tiles that could not be downloaded, 2 when the arguments cannot be parsed.
#![forbid(unsafe_code)]
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented
)]

use std::env;
use std::num::NonZeroUsize;
use std::path::PathBuf;
use std::process::ExitCode;
use std::thread;

use clap::{ArgAction, ArgGroup, Parser};
use tracing::level_filters::LevelFilter;
use tracing::{Level, error, warn};
use tracing_subscriber::filter::Targets;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use valhalla_tools::elevation::{self, Compression, Options, Source};
use valhalla_tools::logging::{self, LineFormat, LogFormat, Logger};

/// Python's `CRITICAL`: shown whatever the verbosity.
const CRITICAL: &str = elevation::CRITICAL_TARGET;

/// Events of this target are logged whatever the verbosity, as the C++ logger does.
const UNFILTERED: &str = "valhalla_build_elevation::unfiltered";

/// Replaces the elevation tiles URL, for tests and mirrors.
const ELEVATION_TILES_URL_VAR: &str = "VALHALLA_ELEVATION_TILES_URL";

#[derive(Parser)]
#[command(
    about = "Downloads Tilezen's elevation tiles that either intersect with features in all \
             .geojson files in --from-geojson, that intersect with --from-bbox, or that are \
             covered by the Valhalla graph with --from-tiles.",
    infer_long_args = true,
    args_override_self = true,
    after_long_help = "Environment:\n  VALHALLA_ELEVATION_TILES_URL  Replaces the tiles URL \
                       (https://elevation-tiles-prod.s3.us-east-1.amazonaws.com/skadi), for \
                       mirrors; empty means unset.",
    group(ArgGroup::new("method").args(["from_geojson", "from_bbox", "from_tiles"])),
    group(ArgGroup::new("compression").args(["decompress", "lz4"]))
)]
struct Cli {
    /// Absolute or relative path to directory with .geojson files used as input to tile
    /// intersection.
    #[arg(short = 'g', long)]
    from_geojson: Option<PathBuf>,
    /// Bounding box coordinates in the format 'minX,minY,maxX,maxY'.
    #[arg(short = 'b', long, allow_hyphen_values = true)]
    from_bbox: Option<String>,
    /// Only download tiles covered by the Valhalla graph (requires a Valhalla config JSON file).
    #[arg(short = 't', long)]
    from_tiles: bool,
    /// Absolute or relative path to the Valhalla config JSON. If present, can be used for
    /// download location and bbox.
    #[arg(short, long)]
    config: Option<PathBuf>,
    /// Inline JSON config, will override --config JSON if present.
    #[arg(short, long, default_value = "{}")]
    inline_config: String,
    /// Absolute or relative path to the directory where the tiles will be saved. Overrides
    /// config JSON.
    #[arg(short, long)]
    outdir: Option<PathBuf>,
    /// Number of parallel downloads [default: number of CPUs].
    #[arg(short, long, allow_negative_numbers = true)]
    parallelism: Option<NonZeroUsize>,
    /// Accumulative verbosity flags; -v: INFO, -vv: DEBUG.
    #[arg(short, long = "verbosity", action = ArgAction::Count)]
    verbosity: u8,
    /// If set, downloaded files will be decompressed (default format is gzip).
    #[arg(short, long)]
    decompress: bool,
    /// If set, downloaded files will be recompressed with LZ4. Requires ~12% more disk space
    /// (216GB total) vs gzip, but tiles are several times faster to decompress.
    #[arg(short = 'z', long)]
    lz4: bool,
    /// Log line format: the Python script's, on stderr, or Valhalla's C++ logger's, which also
    /// follows the config's top-level 'logging' section.
    #[arg(long, value_enum, default_value_t = LogFormat::Python)]
    log_format: LogFormat,
}

impl Cli {
    /// The script ignores an empty inline config.
    fn inline_config(&self) -> Option<&str> {
        Some(self.inline_config.as_str()).filter(|text| !text.is_empty())
    }
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let level = match cli.verbosity {
        0 => LevelFilter::OFF,
        1 => LevelFilter::INFO,
        _ => LevelFilter::DEBUG,
    };
    let logger = match logging::logger(cli.log_format, cli.config.as_deref(), cli.inline_config()) {
        Ok(logger) => logger,
        Err(message) => {
            init_logging(level, cli.log_format, Logger::stderr());
            error!(target: CRITICAL, "{message}");
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
            error!(target: CRITICAL, "{error:#}");
            ExitCode::FAILURE
        }
    }
}

fn init_logging(level: LevelFilter, format: LogFormat, logger: Logger) {
    let filter = Targets::new()
        .with_default(level)
        .with_target(CRITICAL, Level::TRACE)
        .with_target(UNFILTERED, Level::TRACE);
    tracing_subscriber::fmt()
        .with_writer(logger.target)
        .with_max_level(Level::TRACE)
        .event_format(LineFormat {
            format,
            color: logger.color,
            critical_target: Some(CRITICAL),
        })
        .finish()
        .with(filter)
        .init();
}

/// The variable's URL, unless it is unset or empty.
fn tiles_url(variable: Option<String>) -> String {
    variable
        .filter(|url| !url.is_empty())
        .unwrap_or_else(|| elevation::ELEVATION_TILES_URL.to_owned())
}

fn run(cli: Cli) -> anyhow::Result<()> {
    let base_url = tiles_url(env::var(ELEVATION_TILES_URL_VAR).ok());
    let parallelism = cli
        .parallelism
        .or_else(|| thread::available_parallelism().ok())
        .unwrap_or(NonZeroUsize::MIN);
    let inline_config = cli.inline_config().map(str::to_owned);
    // Empty values count as not given, as in the script, except paths, which mean ".".
    let source = if let Some(dir) = cli.from_geojson {
        Some(Source::Geojson(dir))
    } else if let Some(bbox) = cli.from_bbox.filter(|bbox| !bbox.is_empty()) {
        Some(Source::Bbox(bbox))
    } else if cli.from_tiles {
        Some(Source::Graph)
    } else {
        None
    };
    let compression = if cli.decompress {
        Compression::Uncompressed
    } else if cli.lz4 {
        Compression::Lz4
    } else {
        Compression::Gzip
    };
    elevation::run(&Options {
        config: cli.config,
        inline_config,
        outdir: cli.outdir,
        source,
        compression,
        parallelism,
        base_url,
        retry_delays: &elevation::RETRY_DELAYS,
    })?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_tiles_url_variable_counts_as_unset() {
        assert_eq!(
            tiles_url(Some(String::new())),
            elevation::ELEVATION_TILES_URL
        );
    }

    #[test]
    fn a_tiles_url_variable_replaces_the_default() {
        assert_eq!(tiles_url(Some("http://mirror".to_owned())), "http://mirror");
    }
}
