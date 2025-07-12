# -*- coding: utf-8 -*-

import unittest
from valhalla.utils.graph_utils import GraphId



class TestBindings(unittest.TestCase):
    def test_constructors(self):
        g1 = GraphId()
        self.assertFalse(g1.Is_Valid())

        g2 = GraphId(674464002)
        g3 = GraphId("2/421920/20")
        g4 = GraphId(421920, 2, 20)

        self.assertTrue(g2.Is_Valid())
        self.assertTrue(g2 == g3 == g4)
