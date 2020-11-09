#!/usr/bin/env python3
import sys
import json
import math
import random

# TODO: use argparse to parse other request options and validate quadruplets

# generate a certain number of random lls around a center ll with a certain radius
def get_random_locs(yc, xc, radius, count):
    locs = []
    rads = radius / 110567.0
    for i in range(0, count):
        a = float(random.uniform(0.0, 1.0))
        b = float(random.uniform(0.0, 1.0))
        r = rads * math.sqrt(a)
        c = 2 * math.pi * b
        x = r * math.cos(c)
        y = r * math.sin(c)
        x += xc
        y += yc
        locs.append({'lon':round(x, 6),'lat':round(y, 6)});
    return locs


# do each input quadruplet
locs = []
for arg in sys.argv[1:]:
    y, x, radius, count = arg.split(',')
    locs.extend(get_random_locs(float(y), float(x), float(radius), int(count)))

# make requests
for start in locs:
    for end in locs:
        if start == end:
            continue
        req = {'locations':[start, end], 'costing': 'auto'}
        print(json.dumps(req, separators=(',', ':')))
