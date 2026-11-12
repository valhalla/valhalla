//! Downloads the SRTM-style elevation tiles (`.hgt`) that Valhalla's skadi reads, from the
//! public AWS Terrain Tiles bucket.

mod download;
mod error;
mod tiles;

use std::num::NonZeroUsize;
use std::path::{Path, PathBuf};
use std::time::Duration;

use serde_json::{Map, Value};
use tracing::{debug, info};

pub use download::{
    CRITICAL_TARGET, Compression, DownloadOptions, ELEVATION_TILES_URL, RETRY_DELAYS, Summary,
    download,
};
pub use error::Error;
pub use tiles::Tile;

use tiles::{tiles_from_bbox, tiles_from_geojson, tiles_from_graph, tiles_repr};

use crate::paths::python_path;

/// Which tiles to download.
#[derive(Debug, Clone)]
pub enum Source {
    /// Tiles touched by the polygons of the `*.geojson` files in this directory.
    Geojson(PathBuf),
    /// Tiles covered by a `minx,miny,maxx,maxy` bounding box, checked when tiles are selected.
    Bbox(String),
    /// Tiles under the level 2 graph tiles of `mjolnir.tile_dir`.
    Graph,
}

#[derive(Debug, Clone)]
pub struct Options {
    pub config: Option<PathBuf>,
    /// Replaces the config's top-level keys.
    pub inline_config: Option<String>,
    /// Defaults to `additional_data.elevation` of the config.
    pub outdir: Option<PathBuf>,
    pub source: Option<Source>,
    pub compression: Compression,
    pub parallelism: NonZeroUsize,
    pub base_url: String,
    pub retry_delays: &'static [Duration],
}

/// Selects the tiles and downloads them, failing when any could not be downloaded.
pub fn run(options: &Options) -> Result<Summary, Error> {
    let config = crate::config::merge(options.config.as_deref(), options.inline_config.as_deref())?;

    // The script settles the output directory before looking at the tile source.
    let outdir = match &options.outdir {
        Some(outdir) => python_path(outdir),
        None if !config.is_empty() => elevation_dir(&config)?,
        None => return Err(Error::NoOutdir),
    };

    let tiles = match &options.source {
        Some(Source::Geojson(dir)) => tiles_from_geojson(&python_path(dir))?,
        Some(Source::Bbox(bbox)) => tiles_from_bbox(bbox)?,
        Some(Source::Graph) if config.is_empty() => return Err(Error::TilesNeedConfig),
        Some(Source::Graph) => tiles_from_graph(&tile_dir(&config)?)?,
        None => return Err(Error::NoSource),
    };
    debug!("{}", tiles_repr(&tiles));

    let tiles: Vec<_> = tiles.into_iter().collect();
    let summary = download(
        &tiles,
        &DownloadOptions {
            outdir: &outdir,
            compression: options.compression,
            parallelism: options.parallelism,
            base_url: &options.base_url,
            retry_delays: options.retry_delays,
        },
    );
    info!("Downloaded {} tiles. Exiting.", summary.downloaded);
    if summary.failed > 0 {
        return Err(Error::TilesFailed {
            failed: summary.failed,
            total: tiles.len(),
        });
    }
    Ok(summary)
}

/// `config["additional_data"]["elevation"] or "elevation"`.
fn elevation_dir(config: &Map<String, Value>) -> Result<PathBuf, Error> {
    const KEY: &str = "additional_data.elevation";
    let value = config
        .get("additional_data")
        .and_then(|section| section.get("elevation"))
        .ok_or(Error::MissingConfigKey(KEY))?;
    if is_python_falsy(value) {
        return Ok(PathBuf::from("elevation"));
    }
    value
        .as_str()
        .map(|dir| python_path(Path::new(dir)))
        .ok_or(Error::InvalidConfigValue(KEY))
}

fn tile_dir(config: &Map<String, Value>) -> Result<PathBuf, Error> {
    const KEY: &str = "mjolnir.tile_dir";
    let value = config
        .get("mjolnir")
        .and_then(|section| section.get("tile_dir"))
        .ok_or(Error::MissingConfigKey(KEY))?;
    value
        .as_str()
        .map(|dir| python_path(Path::new(dir)))
        .ok_or(Error::InvalidConfigValue(KEY))
}

fn is_python_falsy(value: &Value) -> bool {
    match value {
        Value::Null => true,
        Value::Bool(value) => !value,
        Value::Number(number) => number.as_f64() == Some(0.0),
        Value::String(text) => text.is_empty(),
        Value::Array(items) => items.is_empty(),
        Value::Object(map) => map.is_empty(),
    }
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use serde_json::json;

    use super::*;

    fn object(value: Value) -> Map<String, Value> {
        match value {
            Value::Object(map) => map,
            _ => Map::new(),
        }
    }

    #[test]
    fn the_elevation_dir_comes_from_additional_data() {
        let config = object(json!({"additional_data": {"elevation": "/data/elevation/"}}));
        assert_eq!(
            elevation_dir(&config).unwrap(),
            PathBuf::from("/data/elevation/")
        );
    }

    #[test]
    fn an_empty_or_null_elevation_dir_falls_back_to_elevation() {
        let dirs = [json!(""), json!(null)].map(|value| {
            elevation_dir(&object(json!({"additional_data": {"elevation": value}}))).unwrap()
        });
        assert_eq!(
            dirs,
            [PathBuf::from("elevation"), PathBuf::from("elevation")]
        );
    }

    #[test]
    fn a_config_without_additional_data_is_an_error() {
        let message = elevation_dir(&object(json!({"mjolnir": {}})))
            .unwrap_err()
            .to_string();
        assert_eq!(
            message,
            "the configuration has no 'additional_data.elevation'"
        );
    }

    #[test]
    fn a_non_string_elevation_dir_is_an_error() {
        let config = object(json!({"additional_data": {"elevation": 3}}));
        assert!(matches!(
            elevation_dir(&config),
            Err(Error::InvalidConfigValue(_))
        ));
    }

    #[test]
    fn a_config_without_tile_dir_is_an_error() {
        let config = object(json!({"mjolnir": {"tile_extract": "tiles.tar"}}));
        assert!(matches!(
            tile_dir(&config),
            Err(Error::MissingConfigKey("mjolnir.tile_dir"))
        ));
    }

    #[test]
    fn a_null_tile_dir_is_an_error() {
        let config = object(json!({"mjolnir": {"tile_dir": null}}));
        assert!(matches!(
            tile_dir(&config),
            Err(Error::InvalidConfigValue(_))
        ));
    }

    #[test]
    fn an_empty_tile_dir_is_the_current_directory() {
        let config = object(json!({"mjolnir": {"tile_dir": ""}}));
        assert_eq!(tile_dir(&config).unwrap(), PathBuf::from("."));
    }
}
