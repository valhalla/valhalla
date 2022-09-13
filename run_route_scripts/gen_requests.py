#!/usr/bin/env python3
import sys
import json
import math
import random

# example usage, each arg is a comma separated quadruplet of lon,lat,radius_meters,count
#./gen_requests.py 40.2690,-76.8713,10000,10 39.9561,-76.7313,10000,10 40.0507,-76.3083,10000,10 40.3319,-75.9402,10000,10 40.5910,-75.4788,10000,10 40.0108,-75.2316,10000,10 > a.txt

# example for multipoint routes
#./gen_requests.py 40.2690,-76.8713,1000,2,through 35.9825,-83.9246,1000,2,through 39.775,-105.007,1000,2,through 37.7653,-122.4316,1000,2,through 52.590,13.381,1000,2,through > multi_point_through_auto_routes.txt

# TODO: use argparse to parse other request options and validate quadruplets

# generate a certain number of random lls around a center ll with a certain radius
def get_random_locs(yc, xc, radius, count, waypoint_type):
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
        locs.append({'lon':round(x, 6),'lat':round(y, 6),'type':waypoint_type});
    return locs
    
# do each input quadruplet
locs = []
intermediate_locs = []
for arg in sys.argv[1:]:
    if (len(arg.split(',')) > 4):
        y, x, radius, count, waypoint_type = arg.split(',')
        locs.extend(get_random_locs(float(y), float(x), float(radius), int(count), "break"))
        intermediate_locs.extend(get_random_locs(float(y), float(x),   float(radius), int(count), str(waypoint_type)))
    else:
        y, x, radius, count = arg.split(',')
        locs.extend(get_random_locs(float(y), float(x), float(radius), int(count), "break"))

# make requests
if (intermediate_locs):
    for start in locs:
        for intermediates in intermediate_locs:
            for end in locs:
                if start == end:
                    continue
                req = {'locations':[start, intermediates, end], 'costing': 'auto', 'shape_format': 'polyline6', 'format':'osrm'}
                #req = {'locations':[start, intermediates, end], 'costing': 'bicycle'}
                #req = {'locations':[start, intermediates, end], 'costing': 'pedestrian'}
                print(json.dumps(req, separators=(',', ':')))
else:
    for start in locs:
        for end in locs:
            if start == end:
                continue
            req = {'locations':[start, end], 'costing': 'auto', 'shape_format': 'polyline6', 'format':'osrm'}
            #req = {'locations':[start, end], 'costing': 'bicycle'}
            #req = {'locations':[start, end], 'costing': 'pedestrian'}
            print(json.dumps(req, separators=(',', ':')))
