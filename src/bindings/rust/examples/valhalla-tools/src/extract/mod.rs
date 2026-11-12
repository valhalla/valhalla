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

use std::path::{Path, PathBuf};

use tracing::{debug, info};

pub use error::Error;
pub use geofabrik::{GEOFABRIK_INDEX_URL, region_list};
pub use geometry::{Rect, parse_bbox};

use archive::WriteMode;
use geometry::Polygon;
use source::{SourceTile, TileSource};

#[derive(Debug, Clone)]
pub enum TileFilter {
    All,
    /// Tiles touching the bounding box.
    Bbox(Rect),
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
    /// Rewrite existing outputs in place instead of replacing them; a failure can leave them partial.
    pub in_place: bool,
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
        TileSource::Tar(tar.clone())
    } else if let Some(dir) = mjolnir.tile_dir.as_ref().filter(|dir| dir.is_dir()) {
        TileSource::Dir(dir.clone())
    } else {
        return Err(Error::NoTileSource);
    };
    debug!(?source, "reading tiles");

    let tiles = source.list_tiles()?;
    if tiles.is_empty() {
        return Err(Error::NoTiles(source.path().to_path_buf()));
    }
    let tiles = select_tiles(tiles, &options.filter, &options.geofabrik_index_url)?;
    if tiles.is_empty() {
        return Err(Error::NoTilesInArea(source.path().to_path_buf()));
    }

    let mode = if options.in_place {
        WriteMode::InPlace
    } else {
        WriteMode::Replace
    };
    if let TileSource::Tar(input) = &source {
        refuse_truncating_input(mode, input, &output)?;
    }
    let archived =
        archive::write_indexed_tar(&output, mode, &tiles, |tile| source.read_tile(tile))?;
    info!(tiles = archived.len(), path = %output.display(), "wrote the tile extract");

    if options.with_traffic {
        let traffic_output = mjolnir
            .traffic_extract
            .unwrap_or_else(|| output.with_file_name("traffic.tar"));
        refuse_truncating_input(mode, &output, &traffic_output)?;
        traffic::write_extract(&output, &archived, &traffic_output, mode)?;
        info!(path = %traffic_output.display(), "wrote the traffic extract");
    }
    Ok(())
}

/// Rewriting a file in place while reading it would destroy the data being read.
fn refuse_truncating_input(mode: WriteMode, input: &Path, output: &Path) -> Result<(), Error> {
    if mode == WriteMode::InPlace && archive::is_same_file(input, output) {
        return Err(Error::InPlaceOverInput {
            input: input.to_path_buf(),
            output: output.to_path_buf(),
        });
    }
    Ok(())
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
        TileFilter::Bbox(bbox) => Area::Rect(*bbox),
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
