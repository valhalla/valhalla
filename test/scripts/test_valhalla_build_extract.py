import json
from math import floor
import tarfile
import unittest
from pathlib import Path
import struct
from typing import List, Tuple
import sys
import os

import valhalla_build_extract
from valhalla_build_extract import TILE_SIZES, Bbox

INDEX_BIN_SIZE = valhalla_build_extract.INDEX_BIN_SIZE
INDEX_BIN_FORMAT = valhalla_build_extract.INDEX_BIN_FORMAT

TILE_PATH = Path('test/data/utrecht_tiles')
EXTRACT_PATH = TILE_PATH.joinpath('tiles.tar')
TRAFFIC_PATH = TILE_PATH.joinpath('traffic.tar')

TAR_PATH_LENGTHS = [6, 6, 9]  # how many leading 0's do we need as tar file name?


def tile_base_to_path(base_x: int, base_y: int, level: int) -> str:
    """Convert a tile's base lat/lon to a relative tile path."""
    tile_size = TILE_SIZES[level]

    # assert we got no bogus tile base..
    assert (base_x + 180) % tile_size == 0 and (
                base_y + 90) % tile_size == 0, f"{base_x}, {base_y} on level {level} failed"

    row = floor((base_y + 90) / tile_size)
    col = floor((base_x + 180) / tile_size)

    tile_id = int((row * 360 / tile_size) + col)

    print("row/col: ", row, ", ", col)

    level_tile_id = level | (tile_id << 3)
    path = str(level) + "{:,}".format(int(pow(10, TAR_PATH_LENGTHS[level])) + tile_id).replace(",", os.sep)[1:]

    return path + ".gph"


class TestBuildExtract(unittest.TestCase):
    def test_tile_intersects_bbox(self):
        # bbox with which to filter the tile paths
        bbox = "10.2,53.9,20,59.2"
        input_paths = set([tile_base_to_path(*input_tuple) for input_tuple in (
            (8, 50, 0),  # barely intersecting in the lower left
            (12, 54, 1),  # contained in bbox
            (20, 59, 2),  # barely intersecting in the upper right
        )])
        out_paths = valhalla_build_extract.get_tiles_with_bbox(input_paths, bbox)
        self.assertSetEqual(input_paths, out_paths)

        bbox = "-20,-59.2,-10.2,-53.9"
        input_paths = set([tile_base_to_path(*input_tuple) for input_tuple in (
            (-20, -59, 2),  # barely intersecting in the lower left
            (-12, -54, 1),  # contained in bbox
            (-12, -62, 0),
        )])
        out_paths = valhalla_build_extract.get_tiles_with_bbox(input_paths, bbox)
        self.assertSetEqual(input_paths, out_paths)

        # don't find the ones not intersecting
        bbox = "0,10,4,14"
        input_paths = set([tile_base_to_path(*input_tuple) for input_tuple in (
            (8, 14, 0),
            (-0.50, 9.75, 2)
        )])
        out_paths = valhalla_build_extract.get_tiles_with_bbox(input_paths, bbox)
        self.assertSetEqual(out_paths, set())

    def test_tile_intersects_geojson(self):
        # create 1 polygon with 2 height & width, should leave out e.g. tile (2,2)
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
                        "coordinates": [
                            [[0, 2], [0, 3.99], [0.99, 3.99], [0.99, 2.99], [1.99, 2.99], [1.99, 2], [0, 2]]]
                    }
                }
            ]
        }
        gj_dir = TILE_PATH.joinpath("test_build_extract")
        gj_dir.mkdir()
        gj_fp = gj_dir.joinpath('test_build_extract.geojson')
        with open(gj_fp, 'w') as f:
            json.dump(gj, f)

        input_paths = set([tile_base_to_path(*input_tuple) for input_tuple in (
            (0, 2, 0),
            (0, 2, 1),
            (0, 3, 1),
            (1, 2, 1),
            (0, 2, 2),
            (1.75, 2.75, 2),
            (0.75, 3.25, 2)
        )])
        out_paths = valhalla_build_extract.get_tiles_with_geojson(input_paths, gj_dir)
        print(input_paths, out_paths)
        self.assertSetEqual(input_paths, out_paths)

        # don't find the ones not intersecting
        input_paths = set([tile_base_to_path(*input_tuple) for input_tuple in (
            (4, 2, 0),
            (1, 3, 1),
            (1, 4.75, 2),
            (1, 3, 2),
        )])
        out_paths = valhalla_build_extract.get_tiles_with_geojson(input_paths, gj_dir)
        self.assertSetEqual(out_paths, set())

        gj_fp.unlink()
        gj_dir.rmdir()

    def test_create_extracts(self):
        config = {"mjolnir": {"tile_dir": str(TILE_PATH), "tile_extract": str(EXTRACT_PATH),
                              "traffic_extract": str(TRAFFIC_PATH)}}

        # it will open the tars in write mode, so other test output can't interfere
        tile_paths = sorted(TILE_PATH.rglob('*.gph'))
        valhalla_build_extract.create_extracts(config, True, tile_paths)
        tile_count = len(tile_paths)

        # test that the index has the right offsets/sizes
        exp_tuples = ((2560, 25568, 291912), (296448, 410441, 662496), (960512, 6549282, 6059792))
        self.check_tar(EXTRACT_PATH, exp_tuples, tile_count * INDEX_BIN_SIZE)
        # same for traffic.tar
        exp_tuples = ((1536, 25568, 26416), (28672, 410441, 65552), (95232, 6549282, 604608))
        self.check_tar(TRAFFIC_PATH, exp_tuples, tile_count * INDEX_BIN_SIZE)

    def check_tar(self, p: Path, exp_tuples, end_index):
        with open(p, 'r+b') as f:
            f.seek(tarfile.BLOCKSIZE)
            while f.tell() < end_index + tarfile.BLOCKSIZE:
                t = struct.unpack(INDEX_BIN_FORMAT, f.read(16))
                self.assertIn(t, exp_tuples)


if __name__ == '__main__':
    unittest.main()
