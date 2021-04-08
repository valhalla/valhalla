#!/usr/bin/env python3
import sys
import json
import math
import random

# example usage, each arg is a comma separated quadruplet of lon,lat,radius_meters,count
#./gen_requests.py 40.2690,-76.8713,10000,10 39.9561,-76.7313,10000,10 40.0507,-76.3083,10000,10 40.3319,-75.9402,10000,10 40.5910,-75.4788,10000,10 40.0108,-75.2316,10000,10 > a.txt

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
