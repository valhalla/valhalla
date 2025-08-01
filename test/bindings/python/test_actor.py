# -*- coding: utf-8 -*-

import json
import os
from pathlib import Path
import re
from tempfile import NamedTemporaryFile
import unittest
from valhalla import Actor, get_config, VALHALLA_PYTHON_PACKAGE, VALHALLA_PRINT_VERSION


PWD = Path(os.path.dirname(os.path.abspath(__file__)))

def has_cyrillic(text):
    """
    This is ensuring that the given text contains Cyrillic characters
    :param text:  The text to validate
    :return: Returns true if there are Cyrillic characters
    """
    # Note: The character range includes the entire Cyrillic script range including the extended
    #       Cyrillic alphabet (e.g. ё, Є, ў)
    return bool(re.search('[\u0400-\u04FF]', text))

class TestBindings(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tiles_path = Path('test/data/utrecht_tiles')
        cls.extract_path = Path('test/data/utrecht_tiles/tiles.tar')

        cls.actor = Actor(str(PWD.joinpath('valhalla.json')))

    def test_version_python_package_constant(self):
        self.assertIn("pyvalhalla", VALHALLA_PYTHON_PACKAGE)

        # The CMake build of course doesn't have the setuptools-scm generated __version__.py
        try:
            from valhalla import __version__
            from valhalla.__version__ import __version_tuple__
        except ModuleNotFoundError:
            return
        
        self.assertEqual(".".join([str(x) for x in __version_tuple__[:3]]), VALHALLA_PRINT_VERSION)

        version_modifier = VALHALLA_PRINT_VERSION[VALHALLA_PRINT_VERSION.find("-"):]
        if version_modifier:
            self.assertIn(version_modifier, __version__)
        else:
            # for releases there should always be a version modifier,
            # as they're guaranteed to be run by CI
            self.assertNotEqual("pyvalhalla", VALHALLA_PYTHON_PACKAGE)

    def test_config(self):
        config = get_config(self.extract_path, self.tiles_path)

        self.assertEqual(config['mjolnir']['tile_dir'], str(self.tiles_path.resolve()))
        self.assertEqual(config['mjolnir']['tile_extract'], str(self.extract_path.resolve()))
    
    def test_config_actor(self):
        # shouldn't load the extract, but we cant test that from python
        config = get_config("", self.tiles_path)

        actor = Actor(config)
        self.assertIn('tileset_last_modified', actor.status())
    
    def test_config_no_tiles(self):
        with self.assertRaises(FileNotFoundError):
            get_config("valhalla_tiles")

    def test_route(self):
        query = {
            "locations": [
                {"lat": 52.08813, "lon": 5.03231},
                {"lat": 52.09987, "lon": 5.14913}
            ],
            "costing": "bicycle",
            "directions_options": {"language": "ru-RU"}
        }
        route = self.actor.route(query)

        self.assertIn('trip',  route)
        self.assertIn('units', route['trip'])
        self.assertEqual(route['trip']['units'], 'kilometers')
        self.assertIn('summary', route['trip'])
        self.assertIn('length', route['trip']['summary'])
        self.assertGreater(route['trip']['summary']['length'], .7)
        self.assertIn('legs', route['trip'])
        self.assertGreater(len(route['trip']['legs']), 0)
        self.assertIn('maneuvers', route['trip']['legs'][0])
        self.assertGreater(len(route['trip']['legs'][0]['maneuvers']), 0)
        self.assertIn('instruction', route['trip']['legs'][0]['maneuvers'][0])
        self.assertTrue(has_cyrillic(route['trip']['legs'][0]['maneuvers'][0]['instruction']))

        # test the str api
        route_str = self.actor.route(json.dumps(query, ensure_ascii=False))
        self.assertIsInstance(route_str, str)

        # C++ JSON string has no whitespace, so need to make it json-y
        self.assertEqual(json.dumps(route), json.dumps(json.loads(route_str)))

    def test_isochrone(self):
        query = {
            "locations": [
                {"lat": 52.08813, "lon": 5.03231}
            ],
            "costing": "pedestrian",
            "contours": [
                    {
                        'time': 1
                    }, {
                        'time': 5
                    }, {
                        'distance': 1
                    }, {
                        'distance': 5
                    }
            ],
            "show_locations": True
        }

        iso = self.actor.isochrone(query)
        self.assertEqual(len(iso['features']), 6)  # 4 isochrones and the 2 point layers

    def test_change_config(self):
        with NamedTemporaryFile('w+') as tmp:
            config = get_config(self.extract_path, self.tiles_path)
            config['service_limits']['bicycle']['max_distance'] = 1
            json.dump(config, tmp, indent=2)

            tmp.seek(0)

            actor = Actor(str(tmp.name))

            with self.assertRaises(RuntimeError) as e:
                actor.route(json.dumps({"locations":[{"lat":52.08813,"lon":5.03231},{"lat":52.09987,"lon":5.14913}],"costing":"bicycle","directions_options":{"language":"ru-RU"}}))
            self.assertIn('exceeds the max distance limit', str(e.exception))
