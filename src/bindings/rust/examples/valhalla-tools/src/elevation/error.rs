use std::path::PathBuf;

use crate::python::float_list_repr;

/// Where the script has a message for the same situation, the wording is the script's.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Config(#[from] crate::config::Error),
    #[error(transparent)]
    GeoJson(#[from] crate::geojson::Error),
    #[error("Either config or outdir is required.")]
    NoOutdir,
    #[error("No download method specified.")]
    NoSource,
    #[error("--from-tiles requires a config to be specified.")]
    TilesNeedConfig,
    #[error("the configuration has no '{0}'")]
    MissingConfigKey(&'static str),
    #[error("'{0}' must be a string")]
    InvalidConfigValue(&'static str),
    #[error("BBOX {0} is not a comma-separated string of coordinates.")]
    BboxNotNumbers(String),
    #[error("Bbox invalid: {}", float_list_repr(.0))]
    InvalidBbox(Vec<f64>),
    #[error("Graph directory {} does not exist.", .0.display())]
    NoGraphDir(PathBuf),
    #[error("Couldn't get a tile ID from {}", .0.display())]
    InvalidTilePath(PathBuf),
    #[error("{failed} of {total} elevation tiles could not be downloaded.")]
    TilesFailed { failed: usize, total: usize },
}
