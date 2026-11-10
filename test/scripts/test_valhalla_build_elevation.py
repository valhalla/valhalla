import json
import sys
from pathlib import Path
import unittest
from tempfile import TemporaryDirectory

import valhalla_build_elevation
from valhalla_build_elevation import Tile

# this test suite depends on the utrecht tiles data
TILE_DIR = Path('test/data/utrecht_tiles')


class TestElevationPolygonHoles(unittest.TestCase):
    def test_polygon_holes_exclude_tiles(self):
        outer = [[0, 0], [5, 0], [5, 5], [0, 5], [0, 0]]
        hole = [[1, 1], [4, 1], [4, 4], [1, 4], [1, 1]]
        for geometry in (
            {"type": "Polygon", "coordinates": [outer, hole]},
            {"type": "MultiPolygon", "coordinates": [[outer, hole]]},
        ):
            with self.subTest(type=geometry["type"]), TemporaryDirectory() as tmp:
                geojson = {"type": "FeatureCollection", "features": [{"geometry": geometry}]}
                geojson_path = Path(tmp) / "region.geojson"
                geojson_path.write_text(json.dumps(geojson))

                tiles = valhalla_build_elevation.get_tiles_with_geojson(Path(tmp))

                self.assertNotIn(Tile("N02E002.hgt", "N02"), tiles)
                self.assertIn(Tile("N00E000.hgt", "N00"), tiles)
                self.assertIn(Tile("N02E000.hgt", "N02"), tiles)


class TestBuildElevation(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if not TILE_DIR.exists():
            print(f"{TILE_DIR.resolve()} does not exist.")
            sys.exit(1)

    def test_get_tiles_with_graph(self):
        # gets the tile covering Utrecht
        tiles = valhalla_build_elevation.get_tiles_with_graph(TILE_DIR)
        self.assertEqual(tiles, set([Tile('N52E005.hgt', 'N52')]))

    def test_get_tiles_with_geojson(self):
        # create 1 polygon with 3 height & width, should leave out tile (3,3)
        #  __
        # | |__
        # |___|

        gj = {
            "type": "FeatureCollection",
            "properties": {},
            "features": [
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[[0, 0], [0, 3], [1, 3], [1, 1], [3, 1], [3, 0], [0,0]]]
                    }
                 }
            ]
        }
        gj_dir = TILE_DIR.joinpath("test_build_elevation")
        gj_dir.mkdir(exist_ok=True)
        gj_fp = gj_dir.joinpath('test_build_elevation.geojson')
        with open(gj_fp, 'w') as f:
            json.dump(gj, f)

        tiles = valhalla_build_elevation.get_tiles_with_geojson(gj_dir)
        self.assertEqual(len(tiles), 8)
        self.assertNotIn(Tile("N03E003.hgt", "N03"), tiles)

        gj_fp.unlink()
        gj_dir.rmdir()

    def test_get_tiles_with_bbox(self):
        bbox = "0.90,0.12,3.56,3.12"
        expected_tiles = set([
            Tile("N00E000.hgt", "N00"),
            Tile("N01E000.hgt", "N01"),
            Tile("N02E000.hgt", "N02"),
            Tile("N03E000.hgt", "N03"),
            Tile("N00E001.hgt", "N00"),
            Tile("N01E001.hgt", "N01"),
            Tile("N02E001.hgt", "N02"),
            Tile("N03E001.hgt", "N03"),
            Tile("N00E002.hgt", "N00"),
            Tile("N01E002.hgt", "N01"),
            Tile("N02E002.hgt", "N02"),
            Tile("N03E002.hgt", "N03"),
            Tile("N00E003.hgt", "N00"),
            Tile("N01E003.hgt", "N01"),
            Tile("N02E003.hgt", "N02"),
            Tile("N03E003.hgt", "N03"),
        ])
        tiles = valhalla_build_elevation.get_tiles_with_bbox(bbox)
        self.assertEqual(tiles, expected_tiles)


if __name__ == '__main__':
    unittest.main()
