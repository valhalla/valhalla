import unittest
from pathlib import Path
import valhalla_build_elevation
from valhalla_build_elevation import Tile


class TestBuildElevation(unittest.TestCase):
    def test_get_tile_info(self):
        tile_info_mocks = [
            {"x": 1, "y": 4, "tile_info": Tile("N04E001.hgt", "N04")},
            {"x": -122, "y": 78, "tile_info": Tile("N78W122.hgt", "N78")},
            {"x": 78, "y": -3, "tile_info": Tile("S03E078.hgt", "S03")},
            {"x": -52, "y": -67, "tile_info": Tile("S67W052.hgt", "S67")},
        ]

        for tile_info_mock in tile_info_mocks:
            returned_tile = valhalla_build_elevation.get_tile_info(
                tile_info_mock["x"], tile_info_mock["y"]
            )
            self.assertEqual(returned_tile, tile_info_mock["tile_info"])

    def test_get_tiles_with_shapely(self):
        try:
            import shapely
        except ImportError:
            with self.assertRaises(SystemExit):
                valhalla_build_elevation.get_tiles_with_shapely(Path("mock_dir"))

    def test_get_outer_rings(self):
        pass

    def test_grid_from_bounds(self):
        mock_bounds = [
            {"bounds": [12.5397, 56.15455, 45.74345, 89.97674],
             "yields": 34 * 34, "first": [12, 56, 13, 57], "last": [45, 89, 46, 90]},
            {"bounds": [-45.0023, 67.3168, 2.4502, 69.8999],
             "yields": 49 * 3, "first": [-46, 67, -45, 68], "last": [2, 69, 3, 70]},
            {"bounds": [-0.0023, -87.3168, 0.4502, 89.8999],
             "yields": 2 * 178, "first": [-1, -88, 0, -87], "last": [0, 89, 1, 90]},
        ]

        for mock_bound in mock_bounds:
            grid = list(valhalla_build_elevation.grid_from_bounds(mock_bound["bounds"]))
            yields = len(grid)
            self.assertEqual(yields, mock_bound["yields"])
            self.assertEqual(grid[0], mock_bound["first"])
            self.assertEqual(grid[-1], mock_bound["last"])

    def test_get_tiles_with_bbox(self):
        mock_tiles = [
            {"bbox": "0.90,0.12,3.56,3.12", "tiles": [
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
            ]
            }
        ]
        for mock_tile in mock_tiles:
            tiles = valhalla_build_elevation.get_tiles_with_bbox(mock_tile["bbox"])
            self.assertEqual(tiles, mock_tile["tiles"])

    def test_bbox_is_valid(self):
        mock_bboxes = [
            {"bbox": [12.6345, 67.567, 15.3545, 89.999], "isvalid": True},
            {"bbox": [12.6345, 67.567, 11.3545, 89.999], "isvalid": False},
            {"bbox": [-89.6345, 67.567, 11.3545, 87.568], "isvalid": True},
            {"bbox": [-90.6345, 12.567, 11.3545, 92.568], "isvalid": False},
        ]

        for mock_bbox in mock_bboxes:
            valid = valhalla_build_elevation.bbox_is_valid(mock_bbox["bbox"])
            self.assertEqual(valid, mock_bbox["isvalid"])

    def test_download(self):
        pass


if __name__ == '__main__':
    unittest.main()
