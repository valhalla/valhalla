# -*- coding: utf-8 -*-

import sys
import os
import json
from shutil import rmtree
import valhalla
from valhalla import config

pwd = os.path.dirname(os.path.abspath(__file__))

# Set up and test config
config_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(pwd, 'valhalla.json')
tiles_path = os.path.join(pwd, "valhalla_tiles")
tar_path = os.path.join(pwd, "valhalla_tiles.tar")
valhalla.Configure(
    config_path,
    config.get_default(),
    tiles_path,
    tar_path,
    True
)
with open(config_path) as f:
    config_json = json.load(f)

assert config_json['mjolnir']['tile_dir'] == tiles_path
assert config_json['mjolnir']['tile_extract'] == tar_path

# Build and tar tiles, test
valhalla.BuildTiles(['test/data/utrecht_netherlands.osm.pbf'])
valhalla.TarTiles()

assert all([os.path.isdir(p) for p in [
    tiles_path,
    os.path.join(tiles_path, '0'),
    os.path.join(tiles_path, '1'),
    os.path.join(tiles_path, '2')
]])
assert os.path.isfile(tar_path)

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

# cleanup
rmtree(tiles_path)
os.remove(tar_path)
os.remove(config_path)
