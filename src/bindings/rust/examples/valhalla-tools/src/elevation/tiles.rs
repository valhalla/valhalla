use std::collections::BTreeSet;
use std::fmt;
use std::fs;
use std::path::Path;

use tracing::debug;

use super::Error;
use crate::geojson;
use crate::geometry::Rect;
use crate::paths::resolved;
use crate::python::float_list_repr;

/// Level 2 graph tiles are 0.25 degrees wide, so 1440 to a row.
const LOCAL_SIZE: f64 = 0.25;
const LOCAL_TILES_PER_ROW: u64 = 1440;

/// A 1x1 degree elevation tile: its file name and the directory it is stored in.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Tile {
    /// Such as `N52E005.hgt`, named after its south-west corner.
    pub name: String,
    /// Such as `N52`.
    pub dir: String,
}

impl Tile {
    /// The tile whose south-west corner is at (`x`, `y`) degrees.
    pub fn at(x: i64, y: i64) -> Self {
        let hemisphere = if y < 0 { 'S' } else { 'N' };
        let dir = format!("{hemisphere}{:02}", y.unsigned_abs());
        let east_west = if x < 0 { 'W' } else { 'E' };
        let name = format!("{dir}{east_west}{:03}.hgt", x.unsigned_abs());
        Self { name, dir }
    }
}

/// The script's `namedtuple` repr, for the debug listing.
impl fmt::Display for Tile {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "Tile(name='{}', dir='{}')", self.name, self.dir)
    }
}

/// Python's repr of the tile list, such as `[Tile(name='N52E005.hgt', dir='N52')]`.
pub fn tiles_repr<'a>(tiles: impl IntoIterator<Item = &'a Tile>) -> String {
    let reprs: Vec<_> = tiles.into_iter().map(Tile::to_string).collect();
    format!("[{}]", reprs.join(", "))
}

/// South-west corners of the 1x1 degree cells covering `bounds`, snapped outwards to whole
/// degrees; the cell starting at a whole `max` is excluded.
fn grid(bounds: Rect) -> impl Iterator<Item = (i64, i64)> {
    // `as` saturates, so an absurd coordinate gives a huge grid rather than a wrong one.
    let (min_x, max_x) = (bounds.min_x.floor() as i64, bounds.max_x.ceil() as i64);
    let (min_y, max_y) = (bounds.min_y.floor() as i64, bounds.max_y.ceil() as i64);
    (min_x..max_x).flat_map(move |x| (min_y..max_y).map(move |y| (x, y)))
}

fn cell(x: i64, y: i64) -> Rect {
    Rect {
        min_x: x as f64,
        min_y: y as f64,
        max_x: (x + 1) as f64,
        max_y: (y + 1) as f64,
    }
}

/// Tiles of every cell the `minx,miny,maxx,maxy` bounding box covers. Not extract's
/// `parse_bbox`: each follows its own script, and this one accepts an empty extent.
pub fn tiles_from_bbox(text: &str) -> Result<BTreeSet<Tile>, Error> {
    let not_numbers = || Error::BboxNotNumbers(text.to_owned());
    let values = text
        .split(',')
        .map(python_float)
        .collect::<Option<Vec<_>>>()
        .ok_or_else(not_numbers)?;
    let [min_x, min_y, max_x, max_y] = values[..] else {
        return Err(not_numbers());
    };
    let invalid = min_x > max_x
        || min_x < -180.0
        || max_x > 180.0
        || min_y > max_y
        || min_y < -90.0
        || max_y > 90.0
        || values.iter().any(|value| value.is_nan());
    if invalid {
        return Err(Error::InvalidBbox(values));
    }
    debug!("Received valid bbox: {}", float_list_repr(&values));
    let bounds = Rect {
        min_x,
        min_y,
        max_x,
        max_y,
    };
    Ok(grid(bounds).map(|(x, y)| Tile::at(x, y)).collect())
}

/// Python's `float()`: surrounding whitespace and underscores between digits are allowed.
fn python_float(text: &str) -> Option<f64> {
    let text = text.trim();
    if !text.contains('_') {
        return text.parse().ok();
    }
    let bytes = text.as_bytes();
    let between_digits = bytes.iter().enumerate().all(|(i, &byte)| {
        byte != b'_'
            || (i > 0
                && bytes[i - 1].is_ascii_digit()
                && bytes.get(i + 1).is_some_and(u8::is_ascii_digit))
    });
    if !between_digits {
        return None;
    }
    text.replace('_', "").parse().ok()
}

/// Tiles of the cells touched by the exterior ring of a polygon in the `*.geojson` files
/// directly inside `dir`.
pub fn tiles_from_geojson(dir: &Path) -> Result<BTreeSet<Tile>, Error> {
    let mut tiles = BTreeSet::new();
    for polygon in geojson::read_dir(dir)? {
        let Some(bounds) = polygon.bounds() else {
            continue;
        };
        for (x, y) in grid(bounds) {
            if polygon.intersects(&cell(x, y)) {
                tiles.insert(Tile::at(x, y));
            }
        }
    }
    Ok(tiles)
}

/// Tiles under the level 2 graph tiles (`2/**/*.gph`) of the graph directory.
pub fn tiles_from_graph(graph_dir: &Path) -> Result<BTreeSet<Tile>, Error> {
    if !graph_dir.is_dir() {
        return Err(Error::NoGraphDir(resolved(graph_dir)));
    }
    let local_dir = graph_dir.join("2");
    let mut tiles = BTreeSet::new();
    let mut pending = vec![local_dir.clone()];
    while let Some(dir) = pending.pop() {
        // Like pathlib's `rglob`, which skips what it cannot list and does not follow symlinked
        // directories.
        let Ok(entries) = fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.to_string_lossy().ends_with(".gph") {
                let id = tile_id(&local_dir, &path)
                    .ok_or_else(|| Error::InvalidTilePath(path.clone()))?;
                tiles.insert(graph_tile_cell(id));
            }
            if entry.file_type().is_ok_and(|kind| kind.is_dir()) {
                pending.push(path);
            }
        }
    }
    Ok(tiles)
}

/// The digits of the path below the level directory, such as 818660 for `000/818/660.gph`.
fn tile_id(local_dir: &Path, path: &Path) -> Option<u64> {
    let stem = path.file_name()?.to_str()?.strip_suffix(".gph")?;
    if stem.is_empty() {
        return None;
    }
    let relative = path.strip_prefix(local_dir).ok()?.to_str()?;
    let digits: String = relative
        .strip_suffix(".gph")?
        .chars()
        .filter(|&c| !std::path::is_separator(c))
        .collect();
    if digits.is_empty() || !digits.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    digits.parse().ok()
}

fn graph_tile_cell(id: u64) -> Tile {
    let column = (id % LOCAL_TILES_PER_ROW) as f64;
    let row = (id / LOCAL_TILES_PER_ROW) as f64;
    let x = (column * LOCAL_SIZE - 180.0).floor() as i64;
    let y = (row * LOCAL_SIZE - 90.0).floor() as i64;
    Tile::at(x, y)
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use serde_json::json;

    use super::*;

    fn tile(name: &str) -> Tile {
        Tile {
            name: format!("{name}.hgt"),
            dir: name[..3].to_owned(),
        }
    }

    fn names(tiles: &BTreeSet<Tile>) -> Vec<&str> {
        tiles.iter().map(|tile| tile.name.as_str()).collect()
    }

    #[test]
    fn tiles_are_named_after_their_south_west_corner() {
        assert_eq!(
            [
                Tile::at(5, 52),
                Tile::at(-1, -1),
                Tile::at(-180, 89),
                Tile::at(179, -90)
            ],
            [
                tile("N52E005"),
                tile("S01W001"),
                tile("N89W180"),
                tile("S90E179")
            ]
        );
    }

    #[test]
    fn the_tile_list_is_logged_as_python_namedtuples() {
        let tiles = [Tile::at(0, 47), Tile::at(-1, 48)];
        assert_eq!(
            tiles_repr(&tiles),
            "[Tile(name='N47E000.hgt', dir='N47'), Tile(name='N48W001.hgt', dir='N48')]"
        );
    }

    // Ported from test/scripts/test_valhalla_build_elevation.py.
    #[test]
    fn get_tiles_with_bbox() {
        let tiles = tiles_from_bbox("0.90,0.12,3.56,3.12").unwrap();
        let expected: BTreeSet<_> = (0..4)
            .flat_map(|x| (0..4).map(move |y| Tile::at(x, y)))
            .collect();
        assert_eq!(tiles, expected);
    }

    // Ported from test/scripts/test_valhalla_build_elevation.py.
    #[test]
    fn get_tiles_with_geojson() {
        //  __
        // | |__
        // |___|
        let dir = tempfile::tempdir().unwrap();
        let collection = json!({
            "type": "FeatureCollection",
            "properties": {},
            "features": [{"type": "Feature", "geometry": {
                "type": "Polygon",
                "coordinates": [[[0, 0], [0, 3], [1, 3], [1, 1], [3, 1], [3, 0], [0, 0]]]
            }}]
        });
        let path = dir.path().join("test_build_elevation.geojson");
        fs::write(path, collection.to_string()).unwrap();

        let tiles = tiles_from_geojson(dir.path()).unwrap();
        assert_eq!((tiles.len(), tiles.contains(&tile("N03E003"))), (8, false));
    }

    // Ported from test/scripts/test_valhalla_build_elevation.py, on a fake Utrecht tile tree.
    #[test]
    fn get_tiles_with_graph() {
        let dir = tempfile::tempdir().unwrap();
        for name in ["0/003/196.gph", "1/051/305.gph", "2/000/818/660.gph"] {
            let path = dir.path().join(name);
            fs::create_dir_all(path.parent().unwrap()).unwrap();
            fs::write(path, b"").unwrap();
        }
        let tiles = tiles_from_graph(dir.path()).unwrap();
        assert_eq!(names(&tiles), ["N52E005.hgt"]);
    }

    #[test]
    fn a_bbox_with_negative_values_covers_cells_on_both_sides_of_zero() {
        let tiles = tiles_from_bbox("-0.45,47.56,0.92,48.49").unwrap();
        assert_eq!(
            names(&tiles),
            ["N47E000.hgt", "N47W001.hgt", "N48E000.hgt", "N48W001.hgt"]
        );
    }

    #[test]
    fn a_bbox_on_whole_degrees_excludes_the_cells_starting_at_its_max() {
        assert_eq!(
            names(&tiles_from_bbox("5,52,6,53").unwrap()),
            ["N52E005.hgt"]
        );
    }

    #[test]
    fn a_bbox_with_min_equal_to_max_on_a_whole_degree_covers_nothing() {
        assert!(tiles_from_bbox("5,52,5,53").unwrap().is_empty());
    }

    #[test]
    fn a_bbox_with_min_equal_to_max_inside_a_cell_covers_that_cell() {
        assert_eq!(
            names(&tiles_from_bbox("5.5,52.5,5.5,52.5").unwrap()),
            ["N52E005.hgt"]
        );
    }

    #[test]
    fn bbox_values_accept_spaces_and_underscores_like_python_float() {
        assert_eq!(
            names(&tiles_from_bbox(" 5 ,5_2,\t5.5,52.5 ").unwrap()),
            ["N52E005.hgt"]
        );
    }

    #[test]
    fn a_misplaced_underscore_is_not_a_number() {
        assert!(matches!(
            tiles_from_bbox("5_,52,6,53"),
            Err(Error::BboxNotNumbers(_))
        ));
    }

    #[test]
    fn a_bbox_with_three_values_is_not_coordinates() {
        let message = tiles_from_bbox("1,2,3").unwrap_err().to_string();
        assert_eq!(
            message,
            "BBOX 1,2,3 is not a comma-separated string of coordinates."
        );
    }

    #[test]
    fn a_bbox_with_text_is_not_coordinates() {
        assert!(matches!(
            tiles_from_bbox("1,2,x,4"),
            Err(Error::BboxNotNumbers(_))
        ));
    }

    #[test]
    fn a_reversed_bbox_is_invalid_with_python_reprs() {
        let message = tiles_from_bbox("1,2,0.00001,4").unwrap_err().to_string();
        assert_eq!(message, "Bbox invalid: [1.0, 2.0, 1e-05, 4.0]");
    }

    #[test]
    fn a_bbox_beyond_the_poles_is_invalid() {
        assert!(matches!(
            tiles_from_bbox("0,-91,1,0"),
            Err(Error::InvalidBbox(_))
        ));
    }

    #[test]
    fn a_bbox_beyond_the_antimeridian_is_invalid() {
        assert!(matches!(
            tiles_from_bbox("179,0,180.5,1"),
            Err(Error::InvalidBbox(_))
        ));
    }

    #[test]
    fn a_bbox_with_nan_is_invalid() {
        let message = tiles_from_bbox("nan,0,1,1").unwrap_err().to_string();
        assert_eq!(message, "Bbox invalid: [nan, 0.0, 1.0, 1.0]");
    }

    #[test]
    fn a_bbox_with_infinity_is_invalid() {
        assert!(matches!(
            tiles_from_bbox("-inf,0,1,1"),
            Err(Error::InvalidBbox(_))
        ));
    }

    #[test]
    fn a_polygon_ending_on_a_whole_degree_does_not_select_the_next_cell() {
        let dir = tempfile::tempdir().unwrap();
        let square = json!({"type": "FeatureCollection", "features": [{"geometry": {
            "type": "MultiPolygon",
            "coordinates": [[[[0.2, 0.2], [1, 0.2], [1, 0.8], [0.2, 0.8], [0.2, 0.2]]]]
        }}]});
        fs::write(dir.path().join("a.geojson"), square.to_string()).unwrap();
        assert_eq!(
            names(&tiles_from_geojson(dir.path()).unwrap()),
            ["N00E000.hgt"]
        );
    }

    #[test]
    fn a_missing_graph_dir_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let missing = dir.path().join("tiles");
        let message = tiles_from_graph(&missing).unwrap_err().to_string();
        let expected = format!(
            "Graph directory {} does not exist.",
            std::path::absolute(&missing).unwrap().display()
        );
        assert_eq!(message, expected);
    }

    #[test]
    fn a_graph_dir_without_level_2_has_no_tiles() {
        let dir = tempfile::tempdir().unwrap();
        assert!(tiles_from_graph(dir.path()).unwrap().is_empty());
    }

    #[test]
    fn a_graph_tile_with_a_non_numeric_name_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir_all(dir.path().join("2/000")).unwrap();
        fs::write(dir.path().join("2/000/abc.gph"), b"").unwrap();
        assert!(matches!(
            tiles_from_graph(dir.path()),
            Err(Error::InvalidTilePath(_))
        ));
    }

    #[test]
    fn a_graph_tile_with_an_empty_stem_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir_all(dir.path().join("2/000")).unwrap();
        fs::write(dir.path().join("2/000/.gph"), b"").unwrap();
        assert!(matches!(
            tiles_from_graph(dir.path()),
            Err(Error::InvalidTilePath(_))
        ));
    }

    #[test]
    fn graph_tiles_on_the_south_west_edge_map_to_s90w180() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir_all(dir.path().join("2/000/000")).unwrap();
        fs::write(dir.path().join("2/000/000/000.gph"), b"").unwrap();
        assert_eq!(
            names(&tiles_from_graph(dir.path()).unwrap()),
            ["S90W180.hgt"]
        );
    }
}
