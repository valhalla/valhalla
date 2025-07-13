# -*- coding: utf-8 -*-

import unittest
from valhalla.utils.graph_utils import GraphId, get_tile_base_lon_lat, get_tile_id_from_lon_lat, get_tile_ids_from_bbox


class TestBindings(unittest.TestCase):
    def test_constructors(self):
        g1 = GraphId()
        self.assertFalse(g1.Is_Valid())

        g2 = GraphId(674464002)
        g3 = GraphId("2/421920/20")
        g4 = GraphId(421920, 2, 20)

        self.assertTrue(g2.Is_Valid())
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
        gid_plus = gid_original

        # operator+/operator+=, __add__, __iadd__
        gid_plus += 1
        self.assertEqual(gid_plus, GraphId(gid_original.tileid(), gid_original.level(), gid_original.id() + 1))
        gid_plus = gid_plus + 1
        self.assertEqual(gid_plus, GraphId(gid_original.tileid(), gid_original.level(), gid_original.id() + 2))

        # operator bool, __bool__
        self.assertTrue(gid_original)
        self.assertFalse(GraphId())

    def test_get_tile_base_lon_lat_and_reverse(self):
        gid = GraphId(674464002)
        test_pt = (-180.0, -16.75)

        # happy paths
        self.assertEqual(get_tile_base_lon_lat(gid), test_pt)
        print(test_pt)
        self.assertEqual(get_tile_id_from_lon_lat(gid.level(), test_pt), GraphId(gid.tileid(), gid.level(), 0))

        # exceptions
        with self.assertRaises(ValueError) as exc:
            get_tile_id_from_lon_lat(5, test_pt)
            self.assertNotEqual(exc.msg.find("We only support"))

        with self.assertRaises(ValueError) as exc:
            get_tile_id_from_lon_lat(2, (0, 1, 2))
            self.assertEqual(exc.msg, "Invalid coordinate size, must be 2")

        with self.assertRaises(ValueError) as exc:
            get_tile_id_from_lon_lat(2, (90, 180))
            self.assertEqual(exc.msg, "Invalid coordinate, remember it's (lon, lat)")
    
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
            self.assertNotEqual(exc.msg.find("We only support"))
        
        with self.assertRaises(ValueError) as exc:
            get_tile_ids_from_bbox(-90., -180., 90., 180.)
            self.assertEqual(exc.msg, "Invalid coordinate, remember it's (lon, lat)")        
