# -*- coding: utf-8 -*-

import tempfile
import unittest
from pathlib import Path

from valhalla.baldr import GraphTileHeader
from valhalla.mjolnir import compute_tileset_build_id, set_tileset_build_id

# A real production tile's content hash and the resulting single-tile fold.
BERLIN_TILE_CHECKSUM = 26351030317371
BERLIN_FOLD = 29953


def write_tile(root: Path, rel: str, tile_checksum: int) -> Path:
    """A minimal tile: a valid header claiming the checksum, plus payload."""
    h = GraphTileHeader()
    h.tile_checksum = tile_checksum
    tile = root / rel
    tile.parent.mkdir(parents=True, exist_ok=True)
    tile.write_bytes(h.to_bytes() + b"\x00" * 100)
    return tile


class TestTilesetBuildId(unittest.TestCase):
    def test_small_sums_fold_to_themselves(self):
        with tempfile.TemporaryDirectory() as td:
            write_tile(Path(td), "0/003/015.gph", 5)
            write_tile(Path(td), "1/047/701.gph", 7)
            self.assertEqual(compute_tileset_build_id(td), 12)

    def test_matches_known_real_fold(self):
        with tempfile.TemporaryDirectory() as td:
            write_tile(Path(td), "2/000/820/135.gph", BERLIN_TILE_CHECKSUM)
            self.assertEqual(compute_tileset_build_id(td), BERLIN_FOLD)

    def test_set_stamps_every_tile_and_keeps_checksums(self):
        with tempfile.TemporaryDirectory() as td:
            tiles = [
                write_tile(Path(td), "2/000/820/135.gph", BERLIN_TILE_CHECKSUM),
                write_tile(Path(td), "1/047/701.gph", 1111),
            ]
            expected = compute_tileset_build_id(td)

            set_tileset_build_id(td)

            for tile, checksum in zip(tiles, (BERLIN_TILE_CHECKSUM, 1111)):
                h = GraphTileHeader.from_file(tile)
                self.assertEqual(h.build_id, expected)
                self.assertEqual(h.tile_checksum, checksum)
            # stamping only touches the high bits, so the fold is stable
            self.assertEqual(compute_tileset_build_id(td), expected)


if __name__ == "__main__":
    unittest.main()
