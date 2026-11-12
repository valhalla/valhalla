//! Builds the tile extract (`tiles.tar`) read by Valhalla's `GraphReader`, and optionally an
//! empty `traffic.tar` matching it.

mod archive;
mod config;
mod error;
mod geofabrik;
mod geojson;
mod geometry;
mod source;
mod tile;
mod traffic;

use std::fs;
use std::path::{Path, PathBuf};

use tracing::{debug, info};

pub use archive::WriteMode;
pub use config::section as config_section;
pub use error::Error;
pub use geofabrik::{GEOFABRIK_INDEX_URL, region_list};
pub use geometry::{Rect, parse_bbox};

use geometry::Polygon;
use source::{SourceTile, TileSource};

#[derive(Debug, Clone)]
pub enum TileFilter {
    All,
    /// Tiles touching the `minx,miny,maxx,maxy` bounding box, checked when the tiles are selected.
    Bbox(String),
    /// Tiles touching a polygon of the `*.geojson` files in this directory.
    GeojsonDir(PathBuf),
    /// Tiles touching the boundary of any of these Geofabrik regions.
    Regions(Vec<String>),
}

#[derive(Debug, Clone)]
pub struct Options {
    pub config: Option<PathBuf>,
    pub inline_config: Option<String>,
    /// Output path; with it, an existing `mjolnir.tile_extract` becomes the input.
    pub extract_tar: Option<PathBuf>,
    pub overwrite: bool,
    pub write_mode: WriteMode,
    pub with_traffic: bool,
    pub filter: TileFilter,
    pub geofabrik_index_url: String,
}

pub fn run(options: &Options) -> Result<(), Error> {
    let mjolnir = config::load(options.config.as_deref(), options.inline_config.as_deref())?;

    let output = options
        .extract_tar
        .clone()
        .or_else(|| mjolnir.tile_extract.clone())
        .ok_or(Error::NoOutput)?;
    if output.is_file() && !options.overwrite {
        return Err(Error::OutputExists(output));
    }

    let input_tar = mjolnir
        .tile_extract
        .as_ref()
        .filter(|tar| options.extract_tar.is_some() && tar.is_file());
    let source = if let Some(tar) = input_tar {
        debug!("Using tar file to extract tiles");
        TileSource::Tar(tar.clone())
    } else if let Some(dir) = mjolnir.tile_dir.as_ref().filter(|dir| dir.is_dir()) {
        debug!("Using graph dir to extract tiles");
        TileSource::Dir(dir.clone())
    } else {
        return Err(Error::NoTileSource {
            config: options.config.clone(),
        });
    };

    let traffic_output = options.with_traffic.then(|| {
        mjolnir
            .traffic_extract
            .clone()
            .unwrap_or_else(|| output.with_file_name("traffic.tar"))
    });
    let listed = source.list_tiles()?;
    refuse_overwriting_inputs(&source, &listed, &output, traffic_output.as_deref())?;
    let tiles = select_tiles(listed, &options.filter, &options.geofabrik_index_url)?;
    if tiles.is_empty() {
        return Err(Error::NoTiles(resolved(source.path())));
    }

    // Truncating the input tar would destroy the tiles still to be read, so it is replaced
    // instead; through its canonical path, so that a symlink to it is still written through.
    let (extract_path, extract_mode) = match &source {
        TileSource::Tar(input)
            if options.write_mode == WriteMode::InPlace
                && archive::is_same_file(input, &output) =>
        {
            info!(
                "{} is also the input, writing it through a temporary file",
                output.display()
            );
            let canonical = fs::canonicalize(&output).map_err(Error::io(&output))?;
            (canonical, WriteMode::Atomic)
        }
        _ => (output.clone(), options.write_mode),
    };
    let archived = archive::write_indexed_tar(&extract_path, extract_mode, &tiles, |tile| {
        debug!("Adding tile {} to the tar file", tile.name);
        source.read_tile(tile)
    })?;
    info!(
        "Finished tarring {} tiles to {}",
        archived.len(),
        output.display()
    );

    if let Some(traffic_output) = traffic_output {
        info!("Start creating traffic extract...");
        traffic::write_extract(&output, &archived, &traffic_output, options.write_mode)?;
        info!(
            "Finished creating the traffic extract at {}",
            traffic_output.display()
        );
    }
    Ok(())
}

/// Refuses, before anything is written, outputs that would replace a tile still to be read or
/// the tile extract the traffic extract is built from.
fn refuse_overwriting_inputs(
    source: &TileSource,
    tiles: &[SourceTile],
    output: &Path,
    traffic_output: Option<&Path>,
) -> Result<(), Error> {
    if let TileSource::Dir(_) = source {
        for path in std::iter::once(output).chain(traffic_output) {
            // An output that does not exist yet cannot be one of the tiles.
            if !path.exists() {
                continue;
            }
            if let Some(tile) = tiles
                .iter()
                .find(|tile| archive::is_same_file(&source.file_of(tile), path))
            {
                return Err(Error::OutputIsTile {
                    output: path.to_path_buf(),
                    tile: tile.name.clone(),
                });
            }
        }
    }
    let Some(traffic) = traffic_output else {
        return Ok(());
    };
    let input_tar = match source {
        TileSource::Tar(input) => Some(input.as_path()),
        TileSource::Dir(_) => None,
    };
    for other in std::iter::once(output).chain(input_tar) {
        if archive::is_same_file(traffic, other) || resolved(traffic) == resolved(other) {
            return Err(Error::TrafficOverwrites {
                traffic: traffic.to_path_buf(),
                other: other.to_path_buf(),
            });
        }
    }
    Ok(())
}

/// The absolute path, with symlinks resolved when it exists, like Python's `Path.resolve()`.
fn resolved(path: &Path) -> PathBuf {
    fs::canonicalize(path)
        .or_else(|_| std::path::absolute(path))
        .unwrap_or_else(|_| path.to_path_buf())
}

enum Area {
    Rect(Rect),
    Polygons(Vec<Polygon>),
}

impl Area {
    fn intersects(&self, bounds: &Rect) -> bool {
        match self {
            Self::Rect(rect) => rect.intersects(bounds),
            Self::Polygons(polygons) => polygons.iter().any(|polygon| polygon.intersects(bounds)),
        }
    }
}

fn select_tiles(
    tiles: Vec<SourceTile>,
    filter: &TileFilter,
    geofabrik_index_url: &str,
) -> Result<Vec<SourceTile>, Error> {
    let area = match filter {
        TileFilter::All => return Ok(tiles),
        TileFilter::Bbox(bbox) => Area::Rect(parse_bbox(bbox)?),
        TileFilter::GeojsonDir(dir) => Area::Polygons(geojson::read_dir(dir)?),
        TileFilter::Regions(regions) => {
            Area::Polygons(geofabrik::region_polygons(geofabrik_index_url, regions)?)
        }
    };
    let mut selected = Vec::new();
    for tile in tiles {
        let bounds = tile
            .position
            .bounds()
            .ok_or_else(|| Error::UnknownTileLevel {
                name: tile.name.clone(),
            })?;
        if area.intersects(&bounds) {
            selected.push(tile);
        }
    }
    Ok(selected)
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use serde_json::json;

    use super::*;

    const TILE_SIZES: [f64; 3] = [4.0, 1.0, 0.25];

    /// Bounds of the tile whose south-west corner is at (`x`, `y`) on `level`.
    fn tile_bounds(x: f64, y: f64, level: usize) -> Rect {
        let size = TILE_SIZES[level];
        let row = (y + 90.0) / size;
        let column = (x + 180.0) / size;
        let index = (row * (360.0 / size) + column) as u32;
        let name = format!("{level}/{index}.gph");
        tile::Tile::from_path(&name).unwrap().bounds().unwrap()
    }

    fn touched(area: &Area, tiles: &[(f64, f64, usize)]) -> Vec<bool> {
        tiles
            .iter()
            .map(|&(x, y, level)| area.intersects(&tile_bounds(x, y, level)))
            .collect()
    }

    #[test]
    fn a_bbox_keeps_tiles_it_contains_or_touches() {
        let area = Area::Rect(parse_bbox("10.2,53.9,20,59.2").unwrap());
        assert_eq!(
            touched(&area, &[(8.0, 50.0, 0), (12.0, 54.0, 1), (20.0, 59.0, 2)]),
            [true; 3]
        );
    }

    #[test]
    fn a_bbox_with_negative_coordinates_keeps_tiles_it_touches() {
        let area = Area::Rect(parse_bbox("-20,-59.2,-10.2,-53.9").unwrap());
        assert_eq!(
            touched(
                &area,
                &[(-20.0, -59.0, 2), (-12.0, -54.0, 1), (-12.0, -62.0, 0)]
            ),
            [true; 3]
        );
    }

    #[test]
    fn a_bbox_drops_tiles_outside_it() {
        let area = Area::Rect(parse_bbox("0,10,4,14").unwrap());
        assert_eq!(
            touched(&area, &[(8.0, 14.0, 0), (-0.5, 9.75, 2)]),
            [false; 2]
        );
    }

    fn l_shaped_area() -> Area {
        //  __
        // | |__
        // |___|
        let geometry = json!({
            "type": "Polygon",
            "coordinates": [[[0, 2], [0, 3.99], [0.99, 3.99], [0.99, 2.99], [1.99, 2.99], [1.99, 2], [0, 2]]]
        });
        Area::Polygons(geojson::polygons(&geometry).unwrap())
    }

    #[test]
    fn a_polygon_keeps_tiles_it_overlaps_on_every_level() {
        let tiles = [
            (0.0, 2.0, 0),
            (0.0, 2.0, 1),
            (0.0, 3.0, 1),
            (1.0, 2.0, 1),
            (0.0, 2.0, 2),
            (1.75, 2.75, 2),
            (0.75, 3.25, 2),
        ];
        assert_eq!(touched(&l_shaped_area(), &tiles), [true; 7]);
    }

    #[test]
    fn a_polygon_drops_tiles_in_its_notch_and_outside_it() {
        let tiles = [(4.0, 2.0, 0), (1.0, 3.0, 1), (1.0, 4.75, 2), (1.0, 3.0, 2)];
        assert_eq!(touched(&l_shaped_area(), &tiles), [false; 4]);
    }
}
