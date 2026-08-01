# -*- coding: utf-8 -*-

import io
import tarfile
import tempfile
import unittest
from pathlib import Path

from valhalla.baldr import GraphId, GraphTileHeader
from valhalla.baldr.utils import INDEX_FILE, decode_tar_index, write_tar_index
from valhalla.baldr.utils.tar_index import INDEX_BIN_SIZE

TILES = ("0/003/015.gph", "1/047/701.gph", "2/000/820/135.gph")


def synth_tile(tile_checksum: int, payload_len: int) -> bytes:
    h = GraphTileHeader()
    h.tile_checksum = tile_checksum
    return h.to_bytes() + b"\x00" * payload_len


def build_extract(tar_path: Path, tiles: dict, index_entries: int | None = None) -> None:
    """A tile tar the way valhalla_build_extract lays it out: an index.bin
    placeholder as the first member, then the tiles."""
    with tarfile.open(tar_path, "w") as tar:
        placeholder = tarfile.TarInfo(INDEX_FILE)
        placeholder.size = (len(tiles) if index_entries is None else index_entries) * INDEX_BIN_SIZE
        tar.addfile(placeholder, io.BytesIO(b"\x00" * placeholder.size))
        for rel, data in tiles.items():
            info = tarfile.TarInfo(rel)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))


class TestTarIndex(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.addCleanup(self._td.cleanup)
        self.tar_path = Path(self._td.name) / "tiles.tar"
        self.tiles = {rel: synth_tile(i + 1, 100 * (i + 1)) for i, rel in enumerate(TILES)}

    def test_write_then_read_roundtrip(self):
        build_extract(self.tar_path, self.tiles)

        written = write_tar_index(self.tar_path)

        self.assertEqual(decode_tar_index(self.tar_path), written)
        self.assertEqual(
            [entry.tile_id for entry in written], [GraphId.from_tile_path(rel) for rel in TILES]
        )
        self.assertEqual([entry.size for entry in written], [len(self.tiles[rel]) for rel in TILES])
        # offsets point at each tile's data: reading there yields the tile bytes
        with open(self.tar_path, "rb") as tar_file:
            for entry, rel in zip(written, TILES):
                tar_file.seek(entry.offset)
                self.assertEqual(tar_file.read(entry.size), self.tiles[rel])

    def test_reads_raw_bytes(self):
        build_extract(self.tar_path, self.tiles)
        entries = write_tar_index(self.tar_path)

        # the remote-consumer path: the index fetched as raw bytes (e.g. an
        # HTTP range request), decoded without touching the tar
        with tarfile.open(self.tar_path, "r") as tar:
            raw = tar.extractfile(INDEX_FILE).read()
        self.assertEqual(decode_tar_index(raw), entries)

        with self.assertRaises(ValueError):
            decode_tar_index(raw[:-1])

    def test_rejects_tar_without_leading_index(self):
        with tarfile.open(self.tar_path, "w") as tar:
            rel, data = next(iter(self.tiles.items()))
            info = tarfile.TarInfo(rel)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))

        with self.assertRaises(ValueError):
            decode_tar_index(self.tar_path)
        with self.assertRaises(ValueError):
            write_tar_index(self.tar_path)

    def test_rejects_wrong_size_placeholder(self):
        build_extract(self.tar_path, self.tiles, index_entries=len(self.tiles) - 1)
        with self.assertRaises(ValueError):
            write_tar_index(self.tar_path)


if __name__ == "__main__":
    unittest.main()
