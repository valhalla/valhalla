use super::Error;
use super::geometry::Rect;

// Limits of a GraphId's level and tile index, see baldr/graphid.h and baldr/graphconstants.h.
const MAX_LEVEL: u8 = 7;
const MAX_TILE_INDEX: u32 = 4_194_303;

/// Tile size in degrees per hierarchy level, see baldr/tilehierarchy.cc.
const TILE_SIZES: [f64; 4] = [4.0, 1.0, 0.25, 0.25];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Tile {
    level: u8,
    index: u32,
}

impl Tile {
    /// Parses a path relative to the tile directory, such as `2/000/818/660.gph`.
    pub(crate) fn from_path(name: &str) -> Result<Self, Error> {
        let invalid = || Error::InvalidTilePath {
            name: name.to_owned(),
        };
        let (level, index) = name
            .strip_suffix(".gph")
            .and_then(|stem| stem.split_once('/'))
            .ok_or_else(invalid)?;
        if !is_number(level) || !index.split('/').all(is_number) {
            return Err(invalid());
        }
        let level: u8 = level.parse().map_err(|_| invalid())?;
        let index: u32 = index.replace('/', "").parse().map_err(|_| invalid())?;
        if level > MAX_LEVEL || index > MAX_TILE_INDEX {
            return Err(invalid());
        }
        Ok(Self { level, index })
    }

    /// The tile's GraphId without an object id, as stored in `index.bin`.
    pub(crate) fn id(self) -> u32 {
        u32::from(self.level) | (self.index << 3)
    }

    /// The tile's extent in degrees, `None` for levels without a known tile size.
    pub(crate) fn bounds(self) -> Option<Rect> {
        let size = *TILE_SIZES.get(usize::from(self.level))?;
        let tiles_per_row = 360.0 / size;
        let index = f64::from(self.index);
        let min_x = (index % tiles_per_row) * size - 180.0;
        let min_y = (index / tiles_per_row).floor() * size - 90.0;
        Some(Rect {
            min_x,
            min_y,
            max_x: min_x + size,
            max_y: min_y + size,
        })
    }
}

fn is_number(text: &str) -> bool {
    !text.is_empty() && text.bytes().all(|byte| byte.is_ascii_digit())
}

#[cfg(test)]
#[expect(clippy::unwrap_used, reason = "tests fail loudly on bad fixtures")]
mod tests {
    use super::*;

    #[test]
    fn a_level_0_path_gives_the_level_in_the_low_bits_of_the_id() {
        assert_eq!(Tile::from_path("0/003/196.gph").unwrap().id(), 25568);
    }

    #[test]
    fn a_level_1_path_gives_its_graph_id() {
        assert_eq!(Tile::from_path("1/051/305.gph").unwrap().id(), 410441);
    }

    #[test]
    fn a_level_2_path_with_three_index_parts_gives_its_graph_id() {
        assert_eq!(Tile::from_path("2/000/818/660.gph").unwrap().id(), 6549282);
    }

    #[test]
    fn malformed_tile_paths_are_rejected() {
        let malformed = [
            "index.bin",
            "0/003/196.txt",
            "003196.gph",
            "/0/003/196.gph",
            "0/003//196.gph",
            "0/0a3/196.gph",
            "../003/196.gph",
            "8/000/001.gph",
            "2/004/194/304.gph",
            "0/99999999999.gph",
        ];
        let accepted: Vec<_> = malformed
            .into_iter()
            .filter(|name| Tile::from_path(name).is_ok())
            .collect();
        assert_eq!(accepted, Vec::<&str>::new());
    }

    #[test]
    fn a_level_0_tile_covers_four_degrees() {
        let bounds = Tile::from_path("0/003/196.gph").unwrap().bounds().unwrap();
        assert_eq!(
            bounds,
            Rect {
                min_x: 4.0,
                min_y: 50.0,
                max_x: 8.0,
                max_y: 54.0
            }
        );
    }

    #[test]
    fn a_level_2_tile_covers_a_quarter_degree() {
        let bounds = Tile::from_path("2/000/818/660.gph")
            .unwrap()
            .bounds()
            .unwrap();
        assert_eq!(
            bounds,
            Rect {
                min_x: 5.0,
                min_y: 52.0,
                max_x: 5.25,
                max_y: 52.25
            }
        );
    }

    #[test]
    fn the_first_tile_starts_at_the_south_west_corner_of_the_world() {
        let bounds = Tile::from_path("1/000/000.gph").unwrap().bounds().unwrap();
        assert_eq!((bounds.min_x, bounds.min_y), (-180.0, -90.0));
    }

    #[test]
    fn levels_without_a_tile_size_have_no_bounds() {
        assert_eq!(Tile::from_path("4/000/000.gph").unwrap().bounds(), None);
    }
}
