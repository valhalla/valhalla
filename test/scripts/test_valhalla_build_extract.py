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
    assert (base_x + 180) % tile_size == 0 and (base_y + 90) % tile_size == 0, f"{base_x}, {base_y} failed"

    row = floor((base_y + 90) / tile_size)
    col = floor((base_x + 180) / tile_size)

    tile_id = int((row * 360 / tile_size) + col)

    level_tile_id = level | (tile_id << 3)
    path = str(level)+ "{:,}".format(int(pow(10, TAR_PATH_LENGTHS[level])) + tile_id).replace(",", os.sep)[1:]

    return path + ".gph"


class TestBuildExtract(unittest.TestCase):
    def test_tile_intersects_bbox(self):
        # bbox with which to filter the tile paths
        bbox = Bbox(10.2, 53.9, 20, 59.2)
        for input_tuple in (
            (8, 50, 0),   # barely intersecting in the lower left
            (12, 54, 1),  # contained in bbox
            (20, 59, 2),  # barely intersecting in the upper right
        ):
            tile_path = tile_base_to_path(*input_tuple)
            self.assertTrue(valhalla_build_extract.tile_intersects_bbox(tile_path, bbox), f"Tile {input_tuple} is failing.")

        bbox = Bbox(-20, -59.2, -10.2, -53.9)
        for input_tuple in (
            (-20, -59, 2),  # barely intersecting in the lower left
            (-12, -54, 1),  # contained in bbox
            (-12, -62, 0),
        ):
            tile_path = tile_base_to_path(*input_tuple)
            self.assertTrue(valhalla_build_extract.tile_intersects_bbox(tile_path, bbox), f"Tile {input_tuple} is failing.")

        # don't find the ones not intersecting
        bbox = Bbox(0, 10, 4, 14)
        for input_tuple in (
            (8, 14, 0),
            (-0.50, 9.75, 2)
        ):
            tile_path = tile_base_to_path(*input_tuple)
            self.assertFalse(valhalla_build_extract.tile_intersects_bbox(tile_path, bbox), f"Tile {input_tuple} shouldn't be found.")


    def test_create_extracts(self):
        config = {"mjolnir": {"tile_dir": str(TILE_PATH), "tile_extract": str(EXTRACT_PATH), "traffic_extract": str(TRAFFIC_PATH)}}

        # it will open the tars in write mode, so other test output can't interfere
        valhalla_build_extract.create_extracts(config, True)
        tile_count = valhalla_build_extract.get_tile_count(TILE_PATH)

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
