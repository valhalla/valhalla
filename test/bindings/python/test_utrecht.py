# -*- coding: utf-8 -*-

import os
import json
from shutil import rmtree
from pathlib import Path
import unittest

import valhalla
from valhalla import config

PWD = Path(os.path.dirname(os.path.abspath(__file__)))

class TestBindings(unittest.TestCase):
    """Be a bit lazy: all tests are run in the given order, so that config failure test succeeds."""

    @classmethod
    def setUpClass(cls):
        cls.config_path = os.path.join(PWD, 'valhalla.json')
        cls.tiles_path = os.path.join(PWD, "valhalla_tiles")
        cls.tar_path = os.path.join(PWD, "valhalla_tiles.tar")

    @classmethod
    def tearDownClass(cls):
        rmtree(cls.tiles_path)
        os.remove(cls.tar_path)
        os.remove(cls.config_path)
    
    def test_0_wrong_config_path(self):
        with self.assertRaises(ValueError) as e:
            valhalla.Configure('/highway/to/hell')
            self.assertIn('No local config file found', str(e))

    # Needs to run before configuration was generated the first time
    def test_1_not_configured(self):
        with self.assertRaises(RuntimeError) as e:
            a = valhalla.Actor()
            self.assertIn('The service was not configured', str(e))

    def test_2_no_pbfs(self):
        valhalla.Configure(
            os.path.join(PWD, 'valhalla.json'),
            config=config.get_default(),
            verbose=True
        )
        with self.assertRaises(ValueError) as e:
            valhalla.BuildTiles([])
            self.assertIn('No PBF files', str(e))

    def test_3_invalid_pbfs(self):
        # Valhalla's RuntimError for invalid data
        with self.assertRaises(RuntimeError) as e:
            valhalla.BuildTiles(['blabla'])
            self.assertIn('No PBF files', str(e))

    def test_4_config(self):
        valhalla.Configure(
            self.config_path,
            config.get_default(),
            self.tiles_path,
            self.tar_path,
            True
        )
        with open(self.config_path) as f:
            config_json = json.load(f)

        assert config_json['mjolnir']['tile_dir'] == self.tiles_path
        assert config_json['mjolnir']['tile_extract'] == self.tar_path
    
    def test_5_build_tiles(self):
        pbf_path = os.path.join(PWD.parent.parent, 'data', 'utrecht_netherlands.osm.pbf')
        valhalla.BuildTiles([pbf_path])
        valhalla.TarTiles()

        assert all([os.path.isdir(p) for p in [
            self.tiles_path,
            os.path.join(self.tiles_path, '0'),
            os.path.join(self.tiles_path, '1'),
            os.path.join(self.tiles_path, '2')
        ]])
        assert os.path.isfile(self.tar_path)

    def test_6_route(self):
        actor = valhalla.Actor()
        query = '{"locations":[{"lat":52.08813,"lon":5.03231},{"lat":52.09987,"lon":5.14913}],"costing":"bicycle","directions_options":{"language":"ru-RU"}}'
        route = json.loads(actor.Route(query))

        assert('trip' in route)
        assert('units' in route['trip'] and route['trip']['units'] == 'kilometers')
        assert('summary' in route['trip'] and 'length' in route['trip']['summary'] and route['trip']['summary']['length'] > 9.)
        assert('legs' in route['trip'] and len(route['trip']['legs']) > 0)
        assert('maneuvers' in route['trip']['legs'][0] and len(route['trip']['legs'][0]['maneuvers']) > 0)
        assert('instruction' in route['trip']['legs'][0]['maneuvers'][0])
        assert(route['trip']['legs'][0]['maneuvers'][0]['instruction'] == u'Двигайтесь на восток по велосипедной дорожке.')

    def test_7_change_config(self):
        c = config.get_default()
        c['service_limits']['bicycle']['max_distance'] = 1
        valhalla.Configure(
            self.config_path,
            c,
            self.tiles_path,
            self.tar_path
        )

        actor = valhalla.Actor()
        with self.assertRaises(RuntimeError) as e:
            actor.Route(json.dumps({"locations":[{"lat":52.08813,"lon":5.03231},{"lat":52.09987,"lon":5.14913}],"costing":"bicycle","directions_options":{"language":"ru-RU"}}))
            self.assertIn('exceeds the max distance limit', str(e))
