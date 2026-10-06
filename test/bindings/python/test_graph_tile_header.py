# -*- coding: utf-8 -*-

import tempfile
import unittest
from pathlib import Path

from valhalla.baldr import GraphId, GraphTileHeader

# Real header values.
DATASET_ID = 13283935569
TILE_CHECKSUM = 26351030317371
BUILD_ID = 25426
RAW_CHECKSUM = (BUILD_ID << 48) | TILE_CHECKSUM


def make_header() -> GraphTileHeader:
    h = GraphTileHeader()
    h.dataset_id = DATASET_ID
    h.date_created = 20276
    h.raw_checksum = RAW_CHECKSUM
    return h


class TestGraphTileHeaderBuilder(unittest.TestCase):
    """GraphTileHeader as a standalone, mutable value class with I/O.

    Only the tileset identity fields are writable (dataset_id, date_created,
    raw_checksum) — everything else identifies the tile or is derived from its
    contents and stays read-only.
    """

    def test_default_constructible_and_mutable(self):
        h = make_header()
        # the default constructor stamps the library's own tile version
        self.assertTrue(h.version)
        self.assertEqual(h.dataset_id, DATASET_ID)
        self.assertEqual(h.date_created, 20276)
        # raw_checksum splits into the two derived read-only views
        self.assertEqual(h.raw_checksum, RAW_CHECKSUM)
        self.assertEqual(h.tile_checksum, TILE_CHECKSUM)
        self.assertEqual(h.build_id, BUILD_ID)

    def test_checksum_convenience_setters_keep_the_other_half(self):
        h = make_header()
        h.build_id = 0  # the normalize shape
        self.assertEqual(h.build_id, 0)
        self.assertEqual(h.tile_checksum, TILE_CHECKSUM)

        h.build_id = BUILD_ID  # the stamp shape
        self.assertEqual(h.raw_checksum, RAW_CHECKSUM)

        h.tile_checksum = 1234
        self.assertEqual(h.tile_checksum, 1234)
        self.assertEqual(h.build_id, BUILD_ID)

    def test_checksum_setters_range_checked(self):
        h = make_header()
        with self.assertRaises(ValueError):
            h.tile_checksum = 1 << 48  # more than the 48 hash bits
        with self.assertRaises((TypeError, ValueError, OverflowError)):
            h.build_id = 1 << 16  # more than the 16 build id bits
        # failed sets must not have clobbered anything
        self.assertEqual(h.raw_checksum, RAW_CHECKSUM)

    def test_everything_else_stays_read_only(self):
        h = GraphTileHeader()
        for attr, value in (
            ("graphid", GraphId(820135, 2, 0)),
            ("base_ll", (13.25, 52.5)),
            ("version", "9.9.9"),
            ("nodecount", 1),
            ("directededgecount", 1),
        ):
            with self.assertRaises(AttributeError, msg=attr):
                setattr(h, attr, value)

    def test_bytes_roundtrip(self):
        h = make_header()
        buf = h.to_bytes()
        self.assertEqual(len(buf), GraphTileHeader.byte_size())

        h2 = GraphTileHeader.from_bytes(buf)
        self.assertEqual(h2.graphid, h.graphid)
        self.assertEqual(h2.version, h.version)
        self.assertEqual(h2.dataset_id, h.dataset_id)
        self.assertEqual(h2.date_created, h.date_created)
        self.assertEqual(h2.raw_checksum, h.raw_checksum)
        self.assertEqual(h2.to_bytes(), buf)

    def test_from_bytes_accepts_a_whole_tile_buffer(self):
        # A tile file is header + payload; from_bytes must only look at the
        # header span so callers can hand it a full tile they hold in memory.
        buf = make_header().to_bytes() + b"\xab" * 4096
        h = GraphTileHeader.from_bytes(buf)
        self.assertEqual(h.dataset_id, DATASET_ID)

    def test_file_roundtrip_touches_only_the_header(self):
        payload = bytes(range(256)) * 16
        with tempfile.TemporaryDirectory() as td:
            tile = Path(td) / "135.gph"
            tile.write_bytes(make_header().to_bytes() + payload)

            # the normalize/stamp shape: load, mutate ids, save in place
            h = GraphTileHeader.from_file(tile)
            h.dataset_id = 0
            h.build_id = 0  # keeps the tile_checksum
            h.save(tile)

            reread = GraphTileHeader.from_file(tile)
            self.assertEqual(reread.dataset_id, 0)
            self.assertEqual(reread.build_id, 0)
            self.assertEqual(reread.tile_checksum, TILE_CHECKSUM)
            self.assertEqual(reread.version, GraphTileHeader().version)
            # payload untouched, file size unchanged
            data = tile.read_bytes()
            self.assertEqual(len(data), GraphTileHeader.byte_size() + len(payload))
            self.assertEqual(data[GraphTileHeader.byte_size() :], payload)

    def test_short_buffer_refused(self):
        with self.assertRaises(ValueError):
            GraphTileHeader.from_bytes(b"\x00" * 16)

    def test_short_or_missing_file_refused(self):
        with tempfile.TemporaryDirectory() as td:
            stub = Path(td) / "stub.gph"
            stub.write_bytes(b"\x00" * 16)
            with self.assertRaises(ValueError):
                GraphTileHeader.from_file(stub)
            # save must never grow/create a file: it patches an existing tile
            with self.assertRaises(ValueError):
                make_header().save(stub)
            with self.assertRaises((RuntimeError, ValueError)):
                GraphTileHeader.from_file(Path(td) / "missing.gph")


if __name__ == "__main__":
    unittest.main()
