import tarfile
import unittest
from pathlib import Path
import struct

import valhalla_build_extract

INDEX_BIN_SIZE = valhalla_build_extract.INDEX_BIN_SIZE
INDEX_BIN_FORMAT = valhalla_build_extract.INDEX_BIN_FORMAT

TILE_PATH = Path('test/data/utrecht_tiles')
EXTRACT_PATH = TILE_PATH.joinpath('tiles.tar')
TRAFFIC_PATH = TILE_PATH.joinpath('traffic.tar')


class TestBuildExtract(unittest.TestCase):
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
