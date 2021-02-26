# -*- coding: utf-8 -*-

import sys
import os
import valhalla
import json
import re


def has_cyrillic(text):
    """
    This is ensuring that the given text contains cyrllic characters
    :param text:  The text to validate
    :return: Returns true if there are cyrillic characters
    """
    # Note: The character range includes the entire Cyrillic script range including the extended
    #       Cyrillic alphabet (e.g. ё, Є, ў)
    return bool(re.search('[\u0400-\u04FF]', text))


valhalla.Configure(sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__)) + '/valhalla.json')
actor = valhalla.Actor()
query = '{"locations":[{"lat":52.08813,"lon":5.03231},{"lat":52.09987,"lon":5.14913}],"costing":"bicycle","directions_options":{"language":"ru-RU"}}'
route = json.loads(actor.Route(query))

assert('trip' in route)
assert('units' in route['trip'] and route['trip']['units'] == 'kilometers')
assert('summary' in route['trip'] and 'length' in route['trip']['summary'] and route['trip']['summary']['length'] > 9.)
assert('legs' in route['trip'] and len(route['trip']['legs']) > 0)
assert('maneuvers' in route['trip']['legs'][0] and len(route['trip']['legs'][0]['maneuvers']) > 0)
assert('instruction' in route['trip']['legs'][0]['maneuvers'][0])
assert(has_cyrillic(route['trip']['legs'][0]['maneuvers'][0]['instruction']))
