# -*- coding: utf-8 -*-

"""This needs to be tested completely separate so "no config" tests pass"""

import os
import unittest
from pathlib import Path

import valhalla
from valhalla import config

PWD = Path(os.path.dirname(os.path.abspath(__file__)))


class TestFailures(unittest.TestCase):
    """Also relies on the order such that configuration is done after "no config" tests"""
    def test_0_wrong_config_path(self):
        with self.assertRaises(ValueError):
            valhalla.Configure('/highway/to/hell')

    def test_1_not_configured(self):
        with self.assertRaises(RuntimeError):
            a = valhalla.Actor()

    def test_2_no_pbfs(self):
        valhalla.Configure(
            os.path.join(PWD, 'valhalla.json'),
            config=config.get_default(),
            verbose=False
        )
        with self.assertRaises(ValueError):
            valhalla.BuildTiles([])

    def test_3_invalid_pbfs(self):
        # Valhalla's RuntimError for invalid data
        with self.assertRaises(RuntimeError):
            valhalla.BuildTiles(['blabla'])
