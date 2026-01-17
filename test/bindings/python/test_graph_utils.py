# -*- coding: utf-8 -*-

import copy
import json
import pickle
import unittest
from pathlib import Path

from valhalla.utils import (
    GraphId,
    GraphUtils,
    get_tile_base_lon_lat,
    get_tile_id_from_lon_lat,
    get_tile_ids_from_bbox,
)


class TestBindings(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tiles_path = Path('test/data/utrecht_tiles')
        cls.utrecht_lon, cls.utrecht_lat = 5.03231, 52.08813
        cls.level = 2  # Local roads level
        cls.utrecht_bbox = (5.0, 52.0, 6.0, 53.0)

        # Compute tile GraphId from Utrecht coordinates
        cls.utrecht_tile_gid = get_tile_id_from_lon_lat(cls.level, (cls.utrecht_lon, cls.utrecht_lat))
        cls.utrecht_base_coords = get_tile_base_lon_lat(cls.utrecht_tile_gid)
        cls.utrecht_test_gid = GraphId(cls.utrecht_tile_gid.tileid(), cls.level, 20)

    def test_constructors(self):
        g1 = GraphId()
        self.assertFalse(g1.is_valid())

        g2 = GraphId(self.utrecht_test_gid.value)
        g3 = GraphId(f"{self.level}/{self.utrecht_tile_gid.tileid()}/20")
        g4 = GraphId(self.utrecht_tile_gid.tileid(), self.level, 20)

        self.assertTrue(g2.is_valid())
        # also tests operator==, __eq__
        self.assertTrue(g2 == g3 == g4)

    def test_value(self):
        g = GraphId(self.utrecht_test_gid.value)
        self.assertEqual(self.utrecht_test_gid.value, g.value)
        with self.assertRaises(AttributeError):
            g.value = 0

    def test_operators(self):
        gid_original = GraphId(self.utrecht_test_gid.value)
        # need to copy explicitly
        gid_plus = copy.copy(gid_original)

        # operator+/operator+=, __add__, __iadd__
        gid_plus += 1
        self.assertNotEqual(gid_plus, gid_original)
        self.assertEqual(
            gid_plus, GraphId(gid_original.tileid(), gid_original.level(), gid_original.id() + 1)
        )
        gid_plus = gid_plus + 1
        self.assertEqual(
            gid_plus, GraphId(gid_original.tileid(), gid_original.level(), gid_original.id() + 2)
        )

        # operator bool, __bool__
        self.assertTrue(gid_original)
        self.assertFalse(GraphId())

    def test_pickling(self):
        gid = GraphId(self.utrecht_test_gid.value)

        pickled = pickle.dumps(gid)
        unpickled = pickle.loads(pickled)

        self.assertEqual(gid, unpickled)

        # shallow copy support
        gid_cp = copy.copy(gid)
        self.assertEqual(gid, gid_cp)
        gid_cp += 1
        self.assertNotEqual(gid, gid_cp)

        # deep copy support (should be same as shallow copy anyways)
        gid_cp = copy.deepcopy(gid)
        self.assertEqual(gid, gid_cp)
        gid_cp += 1
        self.assertNotEqual(gid, gid_cp)

    def test_get_tile_base_lon_lat_and_reverse(self):
        # Independent verification: Utrecht tile 818660 at level 2 maps to (5.0, 52.0)
        utrecht_tile_gid = GraphId(818660, 2, 0)
        utrecht_base_pt = (5.0, 52.0)

        self.assertEqual(get_tile_base_lon_lat(utrecht_tile_gid), utrecht_base_pt)
        self.assertEqual(
            get_tile_id_from_lon_lat(utrecht_tile_gid.level(), utrecht_base_pt),
            GraphId(utrecht_tile_gid.tileid(), utrecht_tile_gid.level(), 0)
        )

        # exceptions
        with self.assertRaises(ValueError) as exc:
            get_tile_id_from_lon_lat(5, utrecht_base_pt)
            self.assertNotEqual(exc.msg.find("We only support"))

        with self.assertRaises(ValueError) as exc:
            get_tile_id_from_lon_lat(2, (0, 1, 2))
            self.assertEqual(exc.msg, "Invalid coordinate size, must be 2")

        with self.assertRaises(ValueError) as exc:
            get_tile_id_from_lon_lat(2, (90, 180))
            self.assertEqual(exc.msg, "Invalid coordinate, remember it's (lon, lat)")

    def test_get_tile_ids_from_bbox(self):
        level_0 = get_tile_ids_from_bbox(*self.utrecht_bbox, [0])
        self.assertEqual(len(level_0), 1)

        level_1 = get_tile_ids_from_bbox(*self.utrecht_bbox, [1])
        self.assertEqual(len(level_1), 4)

        level_2 = get_tile_ids_from_bbox(*self.utrecht_bbox, [2])
        self.assertEqual(len(level_2), 25)

        all_levels = get_tile_ids_from_bbox(*self.utrecht_bbox)
        self.assertEqual(len(level_0) + len(level_1) + len(level_2), len(all_levels))

        all_levels = get_tile_ids_from_bbox(*self.utrecht_bbox, [0, 1, 2])
        self.assertEqual(len(level_0) + len(level_1) + len(level_2), len(all_levels))

        all_levels = get_tile_ids_from_bbox(*self.utrecht_bbox, [])
        self.assertEqual(len(level_0) + len(level_1) + len(level_2), len(all_levels))

        # exceptions
        with self.assertRaises(ValueError) as exc:
            get_tile_ids_from_bbox(*self.utrecht_bbox, [4])
            self.assertNotEqual(exc.msg.find("We only support"))

        with self.assertRaises(ValueError) as exc:
            get_tile_ids_from_bbox(-90.0, -180.0, 90.0, 180.0)
            self.assertEqual(exc.msg, "Invalid coordinate, remember it's (lon, lat)")

    def test_get_edge_shape(self):
        """Test GraphUtils.get_edge_shape with real Utrecht tiles."""
        with self.assertRaises((RuntimeError, FileNotFoundError)):
            GraphUtils("invalid json {]")

        config = json.dumps({"mjolnir": {"tile_dir": str(self.tiles_path)}})
        graph = GraphUtils(config)

        fake_edge_id = GraphId(999999, 2, 0)
        with self.assertRaises(RuntimeError) as exc:
            graph.get_edge_shape(fake_edge_id)
        self.assertIn("Tile not found", str(exc.exception))

        tile_gid = get_tile_id_from_lon_lat(self.level, (self.utrecht_lon, self.utrecht_lat))

        # Find first valid edge (following test/minbb.cc pattern)
        edge_found = False
        for edge_idx in range(20):
            try:
                edge_id = GraphId(tile_gid.tileid(), self.level, edge_idx)
                shape = graph.get_edge_shape(edge_id)

                self.assertIsInstance(shape, list)
                self.assertGreater(len(shape), 0, "Edge shape should have points")

                for point in shape:
                    self.assertIsInstance(point, tuple)
                    self.assertEqual(len(point), 2)
                    lon, lat = point
                    self.assertIsInstance(lon, float)
                    self.assertIsInstance(lat, float)

                    self.assertGreater(lon, 4.0, "Longitude should be east of 4°E")
                    self.assertLess(lon, 6.0, "Longitude should be west of 6°E")
                    self.assertGreater(lat, 51.0, "Latitude should be north of 51°N")
                    self.assertLess(lat, 53.0, "Latitude should be south of 53°N")

                edge_found = True
                break

            except RuntimeError:
                continue

        self.assertTrue(edge_found, "Should find at least one valid edge in Utrecht tile")

        invalid_edge_id = GraphId(tile_gid.tileid(), self.level, 999999)
        with self.assertRaises(RuntimeError):
            graph.get_edge_shape(invalid_edge_id)

    def test_graphutils_dict_config(self):
        """Test GraphUtils initialization with dict config."""
        config_dict = {"mjolnir": {"tile_dir": str(self.tiles_path)}}
        graph = GraphUtils(config_dict)

        # Should be able to query edge shapes
        tile_gid = get_tile_id_from_lon_lat(self.level, (self.utrecht_lon, self.utrecht_lat))
        edge_id = GraphId(tile_gid.tileid(), self.level, 0)

        # Just verify we can create the object and call methods
        try:
            shape = graph.get_edge_shape(edge_id)
            self.assertIsInstance(shape, list)
        except RuntimeError:
            # Edge might not exist, but GraphUtils should be initialized
            pass

    def test_graphutils_path_config(self):
        """Test GraphUtils initialization with Path object."""
        # Use path relative to this test file
        config_path = Path(__file__).parent / "valhalla.json"
        graph = GraphUtils(config_path)

        # Should be able to query edge shapes
        tile_gid = get_tile_id_from_lon_lat(self.level, (self.utrecht_lon, self.utrecht_lat))
        edge_id = GraphId(tile_gid.tileid(), self.level, 0)

        try:
            shape = graph.get_edge_shape(edge_id)
            self.assertIsInstance(shape, list)
        except RuntimeError:
            pass

    def test_graphutils_file_path_string(self):
        """Test GraphUtils initialization with file path string."""
        # Use path relative to this test file
        config_path = str(Path(__file__).parent / "valhalla.json")
        graph = GraphUtils(config_path)

        # Should be able to query edge shapes
        tile_gid = get_tile_id_from_lon_lat(self.level, (self.utrecht_lon, self.utrecht_lat))
        edge_id = GraphId(tile_gid.tileid(), self.level, 0)

        try:
            shape = graph.get_edge_shape(edge_id)
            self.assertIsInstance(shape, list)
        except RuntimeError:
            pass

    def test_graphutils_invalid_config_type(self):
        """Test GraphUtils raises error for invalid config types."""
        with self.assertRaises(AttributeError):
            GraphUtils(12345)

        with self.assertRaises(AttributeError):
            GraphUtils([1, 2, 3])

        with self.assertRaises(AttributeError):
            GraphUtils(None)

    def test_graphutils_missing_tile_data(self):
        """Test GraphUtils raises error when tile data doesn't exist."""
        config = {
            "mjolnir": {
                "tile_extract": "/nonexistent/tiles.tar",
                "tile_dir": "/nonexistent/tiles",
            }
        }

        with self.assertRaises(FileNotFoundError) as exc:
            GraphUtils(config)
        self.assertIn("Can't load graph", str(exc.exception))

    def test_graphutils_missing_mjolnir_config(self):
        """Test GraphUtils raises error when mjolnir config is missing."""
        config = {"loki": {"actions": ["route"]}}

        with self.assertRaises(AttributeError) as exc:
            GraphUtils(config)
        self.assertIn("mjolnir.tile_extract and mjolnir.tile_dir are missing", str(exc.exception))

