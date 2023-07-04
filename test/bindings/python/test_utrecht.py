# -*- coding: utf-8 -*-

import json
import os
from pathlib import Path
import re
import unittest
from valhalla import Actor, get_config


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
        cls.config_path = PWD.joinpath('valhalla.json')
        cls.tiles_path = Path('test/data/utrecht_tiles')
        cls.extract_path = Path('test/data/utrecht_tiles/tiles.tar')

        cls.actor = Actor(str(cls.config_path))
        
        # remember the old config and write it back after the tests
        with open(cls.config_path) as f:
            cls.config = json.load(f)

    @classmethod
    def tearDownClass(cls):
        with open(cls.config_path, 'w') as f:
            json.dump(cls.config, f, indent=2)
        del cls.actor

    def test_config(self):
        config = get_config(self.tiles_path, self.extract_path)

        self.assertEqual(config['mjolnir']['tile_dir'], str(self.tiles_path.resolve()))
        self.assertEqual(config['mjolnir']['tile_extract'], str(self.extract_path.resolve()))
    
    def test_config_actor(self):
        # shouldn't load the extract, but we cant test that from python
        config = get_config(self.tiles_path)
        with open(self.config_path, 'w') as f:
            json.dump(config, f, indent=2)
            
        actor = Actor(str(self.config_path))
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
        config = get_config(self.tiles_path, self.extract_path)
        config['service_limits']['bicycle']['max_distance'] = 1
        with open(self.config_path, 'w') as f:
            json.dump(config, f, indent=2)

        actor = Actor(str(self.config_path))

        with self.assertRaises(RuntimeError) as e:
            actor.route(json.dumps({"locations":[{"lat":52.08813,"lon":5.03231},{"lat":52.09987,"lon":5.14913}],"costing":"bicycle","directions_options":{"language":"ru-RU"}}))
        self.assertIn('exceeds the max distance limit', str(e.exception))
