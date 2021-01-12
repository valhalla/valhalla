# -*- coding: utf-8 -*-

import os
import json
from shutil import rmtree
from pathlib import Path
import unittest

import valhalla
from valhalla import config
from valhalla.utils import decode_polyline

PWD = Path(os.path.dirname(os.path.abspath(__file__)))

class TestBindings(unittest.TestCase):
    """Be a bit lazy: all tests are run in the given order, so that config failure test succeeds."""

    @classmethod
    def setUpClass(cls):
        cls.config_path = Path(os.path.join(PWD, 'valhalla.json'))
        cls.tiles_path = Path(os.path.join(PWD, "valhalla_tiles"))
        cls.tar_path = Path(os.path.join(PWD, "valhalla_tiles.tar"))

    @classmethod
    def tearDownClass(cls):
        delete = [cls.tiles_path, cls.tar_path, cls.config_path]
        d: Path
        for d in delete:
            if d.is_dir():
                rmtree(d)
            elif d.is_file():
                os.remove(d)

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
            str(self.config_path),
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
            str(self.config_path),
            config.get_default(),
            str(self.tiles_path),
            str(self.tar_path),
            True
        )
        with open(self.config_path) as f:
            config_json = json.load(f)

        assert config_json['mjolnir']['tile_dir'] == str(self.tiles_path)
        assert config_json['mjolnir']['tile_extract'] == str(self.tar_path)

        from valhalla.config import _global_config
        assert _global_config['mjolnir']['tile_dir'] == str(self.tiles_path)
        assert _global_config['mjolnir']['tile_extract'] == str(self.tar_path)
    
    def test_5_build_tiles(self):
        pbf_path = os.path.join(PWD.parent.parent, 'data', 'nyc.osm.pbf')
        tar_path = Path(valhalla.BuildTiles([pbf_path]))

        assert tar_path == self.tar_path
        assert tar_path.is_file()
        assert tar_path.stat().st_size > 90000  # actual produced a tar

    def test_6_route(self):
        actor = valhalla.Actor()

        query = {
            "locations": [
                {"lat": 40.75120639, "lon": -74.00242363},
                {"lat": 40.74559857, "lon": -74.00650242}
            ],
            "costing": "bicycle",
            "directions_options": {"language": "ru-RU"}
        }
        route = actor.Route(query)

        assert('trip' in route)
        assert('units' in route['trip'] and route['trip']['units'] == 'kilometers')
        assert('summary' in route['trip'] and 'length' in route['trip']['summary'] and route['trip']['summary']['length'] > .7)
        assert('legs' in route['trip'] and len(route['trip']['legs']) > 0)
        assert('maneuvers' in route['trip']['legs'][0] and len(route['trip']['legs'][0]['maneuvers']) > 0)
        assert('instruction' in route['trip']['legs'][0]['maneuvers'][0])
        assert(route['trip']['legs'][0]['maneuvers'][0]['instruction'] == u'Двигайтесь на юг по Hudson River Greenway.')

        route_str = actor.Route(json.dumps(query, ensure_ascii=False))
        assert isinstance(route_str, str)
        # C++ JSON string has no whitespace, so need to make it jsony
        assert json.dumps(route) == json.dumps(json.loads(route_str))

    def test_7_change_config(self):
        c = config.get_default()
        c['service_limits']['bicycle']['max_distance'] = 1
        valhalla.Configure(
            str(self.config_path),
            c,
            str(self.tiles_path),
            str(self.tar_path)
        )

        actor = valhalla.Actor()
        with self.assertRaises(RuntimeError) as e:
            actor.Route(json.dumps({"locations":[{"lat":52.08813,"lon":5.03231},{"lat":52.09987,"lon":5.14913}],"costing":"bicycle","directions_options":{"language":"ru-RU"}}))
            self.assertIn('exceeds the max distance limit', str(e))

    def test_8_decode_polyline(self):
        encoded = 'mpivlAhwadlCxl@jPhj@hOdJ~BnFjAdEf@pIp@bDHhHKrI[~EB|AG|B_@fNuDzC?bCTzAXtFlBhANnADrAKhA]rAi@|A{@fGkE|CuApDuA|Ac@jAm@lAy@xA_C~@iD`@cD\\mAh@cAv@e@v@UrAOjB@~BNjUzBzz@xIndAnK'

        dec6 = decode_polyline(encoded)
        assert len(dec6) == 43
        assert dec6[0] == (-74.007941, 40.752407)

        dec6_latlng = decode_polyline(encoded, order='latlng')
        assert len(dec6_latlng) == 43
        assert dec6_latlng[0] == tuple(reversed(dec6[0]))
