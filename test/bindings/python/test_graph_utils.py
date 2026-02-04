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
    get_tile_ids_from_ring,
)


class TestBindings(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tiles_path = Path("test/data/utrecht_tiles")
        cls.utrecht_lon, cls.utrecht_lat = 5.03231, 52.08813
        cls.level = 2  # Local roads level

    def test_constructors(self):
        g1 = GraphId()
        self.assertFalse(g1.is_valid())

        g2 = GraphId(674464002)
        g3 = GraphId("2/421920/20")
        g4 = GraphId(421920, 2, 20)

        self.assertTrue(g2.is_valid())
        # also tests operator==, __eq__
        self.assertTrue(g2 == g3 == g4)

    def test_value(self):
        v_test = 674464002
        g = GraphId(v_test)
        self.assertEqual(v_test, g.value)
        with self.assertRaises(AttributeError):
            g.value = 0

    def test_operators(self):
        v_test = 674464002
        gid_original = GraphId(v_test)
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
        gid = GraphId(674464002)

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
        gid = GraphId(674464002)
        test_pt = (-180.0, -16.75)

        self.assertEqual(get_tile_base_lon_lat(gid), test_pt)
        self.assertEqual(
            get_tile_id_from_lon_lat(gid.level(), test_pt), GraphId(gid.tileid(), gid.level(), 0)
        )

        # exceptions
        with self.assertRaises(ValueError) as exc:
            get_tile_id_from_lon_lat(5, test_pt)
        self.assertIn("We only support", str(exc.exception))

        with self.assertRaises(ValueError) as exc:
            get_tile_id_from_lon_lat(2, (0, 1, 2))
        self.assertIn("Invalid coordinate size, must be 2", str(exc.exception))

        with self.assertRaises(ValueError) as exc:
            get_tile_id_from_lon_lat(2, (90, 180))
        self.assertIn("Invalid coordinate, remember it's (lon, lat)", str(exc.exception))

    def test_get_tile_ids_from_bbox(self):
        bbox = (0, 0, 2, 2)

        # happy paths
        level_0 = get_tile_ids_from_bbox(*bbox, [0])
        self.assertEqual(len(level_0), 2)

        level_1 = get_tile_ids_from_bbox(*bbox, [1])
        self.assertEqual(len(level_1), 9)

        level_2 = get_tile_ids_from_bbox(*bbox, [2])
        self.assertEqual(len(level_2), 81)

        all_levels = get_tile_ids_from_bbox(*bbox)
        self.assertEqual(len(level_0) + len(level_1) + len(level_2), len(all_levels))

        all_levels = get_tile_ids_from_bbox(*bbox, [0, 1, 2])
        self.assertEqual(len(level_0) + len(level_1) + len(level_2), len(all_levels))

        all_levels = get_tile_ids_from_bbox(*bbox, [])
        self.assertEqual(len(level_0) + len(level_1) + len(level_2), len(all_levels))

        # exceptions
        with self.assertRaises(ValueError) as exc:
            get_tile_ids_from_bbox(*bbox, [4])
        self.assertIn("We only support", str(exc.exception))

        with self.assertRaises(ValueError) as exc:
            get_tile_ids_from_bbox(-90.0, -180.0, 90.0, 180.0)
        self.assertIn("Invalid coordinate, remember it's (lon, lat)", str(exc.exception))

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

    def test_get_tile_ids_from_ring_rectangle(self):
        """Rectangle ring should match bbox results.
        """
        ring = [(0.5, 0.5), (2.5, 0.5), (2.5, 2.5), (0.5, 2.5)]
        ring_result = get_tile_ids_from_ring(ring, [1])
        bbox_result = get_tile_ids_from_bbox(0.5, 0.5, 2.5, 2.5, [1])

        self.assertEqual(len(ring_result), 9)
        self.assertEqual(
            {r.value for r in ring_result},
            {b.value for b in bbox_result},
        )

    def test_get_tile_ids_from_ring_triangle(self):
        """Triangle should return fewer tiles than its bounding box.
        """
        ring = [(2, 3), (14, 3), (8, 9)]
        ring_result = get_tile_ids_from_ring(ring, [0])
        bbox_result = get_tile_ids_from_bbox(2, 3, 14, 9, [0])

        ring_set = {r.value for r in ring_result}
        bbox_set = {b.value for b in bbox_result}

        self.assertTrue(ring_set)
        self.assertTrue(ring_set <= bbox_set, "Ring tiles must be a subset of bbox tiles")
        self.assertLess(len(ring_set), len(bbox_set), "Triangle must have fewer tiles than bbox")
        self.assertTrue(all(r.level() == 0 for r in ring_result))

    def test_get_tile_ids_from_ring_diamond(self):
        """Diamond tests flood fill of interior tiles."""
        ring = [(4, 0), (8, 4), (4, 8), (0, 4)]
        ring_result = get_tile_ids_from_ring(ring, [1])
        bbox_result = get_tile_ids_from_bbox(0, 0, 8, 8, [1])

        ring_set = {r.value for r in ring_result}
        bbox_set = {b.value for b in bbox_result}

        self.assertTrue(ring_set)
        self.assertTrue(ring_set <= bbox_set)
        self.assertLess(len(ring_set), len(bbox_set))
        self.assertGreater(len(ring_set), 20, "Diamond should contain a decent number of tiles")

    def test_get_tile_ids_from_ring_orientation(self):
        """CW and CCW winding order should produce identical results."""
        cw = [(1, 1), (3, 1), (3, 3), (1, 3)]
        ccw = [(1, 1), (1, 3), (3, 3), (3, 1)]

        cw_result = get_tile_ids_from_ring(cw, [1])
        ccw_result = get_tile_ids_from_ring(ccw, [1])

        self.assertEqual(
            {r.value for r in cw_result},
            {r.value for r in ccw_result}, 
        ) 

    def test_get_tile_ids_from_ring_auto_close(self):
        """Open ring (last != first) should be auto-closed."""
        open_ring = [(0, 0), (4, 0), (2, 3)]
        closed_ring = [(0, 0), (4, 0), (2, 3), (0, 0)]

        open_result = get_tile_ids_from_ring(open_ring, [1])
        closed_result = get_tile_ids_from_ring(closed_ring, [1])

        self.assertEqual(
            {r.value for r in open_result},
            {r.value for r in closed_result},
        )

    def test_get_tile_ids_from_ring_default_levels(self):
        """No levels specified should return tiles at all 3 levels (0, 1, 2)."""
        ring = [(0, 0), (2, 0), (1, 2)]

        result_default = get_tile_ids_from_ring(ring)
        result_explicit = get_tile_ids_from_ring(ring, [0, 1, 2])

        levels_found = {r.level() for r in result_default}
        self.assertEqual(levels_found, {0, 1, 2})

        self.assertEqual(
            {r.value for r in result_default},
            {r.value for r in result_explicit},
        )

    def test_get_tile_ids_from_ring_errors(self):
        """Invalid inputs should raise ValueError."""
        # too few coordinates
        with self.assertRaises(ValueError) as exc:
            get_tile_ids_from_ring([(0, 0), (1, 1)])
        self.assertIn("at least 3", str(exc.exception))

        # invalid level
        with self.assertRaises(ValueError) as exc:
            get_tile_ids_from_ring([(0, 0), (1, 0), (0.5, 1)], [5])
        self.assertIn("We only support", str(exc.exception))

        # invalid coordinate (lat/lon swapped)
        with self.assertRaises(ValueError) as exc:
            get_tile_ids_from_ring([(0, 0), (0, 91), (1, 91)])
        self.assertIn("Invalid coordinate", str(exc.exception))
