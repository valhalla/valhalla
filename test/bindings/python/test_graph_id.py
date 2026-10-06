# -*- coding: utf-8 -*-

import os
import unittest

from valhalla.baldr import GraphId


class TestFromTilePath(unittest.TestCase):
    """GraphId.from_tile_path — the inverse of os.fspath(graph_id)."""

    def test_parses_tile_paths(self):
        for path, expected in (
            ("2/000/820/135.gph", GraphId(820135, 2, 0)),
            ("1/047/701.gph", GraphId(47701, 1, 0)),
            ("0/003/015.gph", GraphId(3015, 0, 0)),
        ):
            self.assertEqual(GraphId.from_tile_path(path), expected)

    def test_absolute_paths_and_extensions(self):
        self.assertEqual(
            GraphId.from_tile_path("/data/valhalla_tiles/2/000/820/135.gph"),
            GraphId(820135, 2, 0),
        )
        self.assertEqual(GraphId.from_tile_path("2/000/820/135.gph.gz"), GraphId(820135, 2, 0))

    def test_inverse_of_fspath(self):
        gid = GraphId(820135, 2, 0)
        self.assertEqual(GraphId.from_tile_path(os.fspath(gid)), gid)

    def test_rejects_invalid_paths(self):
        for path in ("135.gph", "nonsense", "2/000/82x/135.gph", ""):
            with self.assertRaises(RuntimeError):
                GraphId.from_tile_path(path)


if __name__ == "__main__":
    unittest.main()
