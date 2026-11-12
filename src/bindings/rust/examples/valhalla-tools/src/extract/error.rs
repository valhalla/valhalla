use std::io;
use std::path::PathBuf;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("no configuration given, pass --config or --inline-config")]
    NoConfig,
    #[error("invalid --inline-config JSON")]
    InlineConfig(#[source] serde_json::Error),
    #[error("{0} must be a JSON object")]
    ConfigNotObject(String),
    #[error("the configuration has no 'mjolnir' object")]
    MissingMjolnir,
    #[error("'mjolnir.{key}' must be a string")]
    InvalidConfigValue { key: &'static str },
    #[error("no output path, set 'mjolnir.tile_extract' or pass --extract-tar")]
    NoOutput,
    #[error("{} exists, pass --overwrite to replace it", .0.display())]
    OutputExists(PathBuf),
    #[error(
        "no tiles to read: 'mjolnir.tile_dir' is not a directory and \
         'mjolnir.tile_extract' is not an existing tar (or --extract-tar was not given)"
    )]
    NoTileSource,
    #[error("invalid bounding box '{text}': {reason}")]
    InvalidBbox { text: String, reason: &'static str },
    #[error("'{name}' is not a tile path such as 2/000/818/660.gph")]
    InvalidTilePath { name: String },
    #[error("tile {name} is on a level without a known tile size")]
    UnknownTileLevel { name: String },
    #[error("no tiles found in {}", .0.display())]
    NoTiles(PathBuf),
    #[error("no tile in {} matches the requested area", .0.display())]
    NoTilesInArea(PathBuf),
    #[error(
        "--in-place would truncate {} while reading it through {}; choose another output or drop --in-place",
        .output.display(),
        .input.display()
    )]
    InPlaceOverInput { input: PathBuf, output: PathBuf },
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
    #[error("invalid JSON in {}", path.display())]
    Json {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error("{} is not a directory holding .geojson files", .0.display())]
    NoGeoJson(PathBuf),
    #[error("{}: expected a FeatureCollection with valid Polygon or MultiPolygon geometries", .0.display())]
    InvalidGeoJson(PathBuf),
    #[error("failed to download {url}")]
    Download {
        url: String,
        #[source]
        source: Box<ureq::Error>,
    },
    #[error("{url} answered with HTTP status {status}")]
    HttpStatus { url: String, status: u16 },
    #[error("invalid Geofabrik index at {url}")]
    InvalidIndex {
        url: String,
        #[source]
        source: serde_json::Error,
    },
    #[error("region '{0}' not found, use '--region list' to see the available regions")]
    UnknownRegion(String),
    #[error("region '{0}' has no valid polygon geometry")]
    InvalidRegionGeometry(String),
}

impl Error {
    pub(crate) fn io(path: impl Into<PathBuf>) -> impl FnOnce(io::Error) -> Self {
        let path = path.into();
        move |source| Self::Io { path, source }
    }
}
