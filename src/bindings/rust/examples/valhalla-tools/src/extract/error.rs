use std::io;
use std::path::{Path, PathBuf};

use crate::python::float_list_repr;

/// Where the script has a message for the same situation, the wording is the script's.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("No valid config file or inline config used.")]
    NoConfig,
    #[error(transparent)]
    Config(#[from] crate::config::Error),
    #[error(transparent)]
    GeoJson(#[from] crate::geojson::Error),
    #[error("the configuration has no 'mjolnir' object")]
    MissingMjolnir,
    #[error("'mjolnir.{key}' must be a string")]
    InvalidConfigValue { key: &'static str },
    #[error("No output file path specified in 'mjolnir.tile_extract' or --extract-tar.")]
    NoOutput,
    #[error("File exists. Specify --overwrite to overwrite {}", .0.display())]
    OutputExists(PathBuf),
    #[error(
        "'Can't find valid paths for 'mjolnir.tile_dir' or 'mjolnir.tile_extract' in {}",
        display_or_none(.config.as_deref())
    )]
    NoTileSource { config: Option<PathBuf> },
    #[error("BBOX {0} is not a comma-separated string of coordinates.")]
    BboxNotNumbers(String),
    #[error("Bbox invalid: {}", float_list_repr(.0))]
    InvalidBbox(Vec<f64>),
    #[error("Couldn't get level and tile ID for tile {name}")]
    InvalidTilePath { name: String },
    #[error("tile {name} is on a level without a known tile size")]
    UnknownTileLevel { name: String },
    #[error("{} is the input tile {tile}; write the extract outside the tile directory", .output.display())]
    OutputIsTile { output: PathBuf, tile: String },
    #[error("the traffic extract {} would overwrite {}; set 'mjolnir.traffic_extract' to another path", .traffic.display(), .other.display())]
    TrafficOverwrites { traffic: PathBuf, other: PathBuf },
    #[error("Couldn't find usable tiles in {}", .0.display())]
    NoTiles(PathBuf),
    #[error("tile {name} has {size} bytes, more than index.bin can describe")]
    TileTooLarge { name: String, size: u64 },
    #[error("tile {name} is too small to hold a graph tile header")]
    TruncatedTileHeader { name: String },
    #[error("failed to copy tile {name} into {}", path.display())]
    CopyTile {
        name: String,
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("I/O error on {}", path.display())]
    Io {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    // The cause is part of the sentence, as in the script, rather than a separate source.
    #[error("Could not fetch Geofabrik index: {reason}")]
    Download {
        url: String,
        reason: Box<ureq::Error>,
    },
    #[error("Could not fetch Geofabrik index: HTTP Error {status}: {reason}")]
    HttpStatus {
        url: String,
        status: u16,
        reason: &'static str,
    },
    #[error("invalid Geofabrik index at {url}")]
    InvalidIndex {
        url: String,
        #[source]
        source: serde_json::Error,
    },
    #[error("Region '{0}' not found. Use '--region list' to see available regions.")]
    UnknownRegion(String),
    #[error("Could not parse geometry for region '{0}'.")]
    InvalidRegionGeometry(String),
}

impl Error {
    pub(crate) fn io(path: impl Into<PathBuf>) -> impl FnOnce(io::Error) -> Self {
        let path = path.into();
        move |source| Self::Io { path, source }
    }
}

fn display_or_none(path: Option<&Path>) -> String {
    path.map_or_else(|| "None".to_owned(), |path| path.display().to_string())
}
