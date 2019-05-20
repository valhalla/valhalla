#!/usr/bin/env python
import sys
import math
import random
import json
import re


### This script can take either a *_locations.txt file generated with randomness
### (created from create_random_points_within_radius) or a manually generated location set.
### The create_routes function with create routes from each location in the list to
### every other location in the list.  You can specify the number of input locations.
### Example: 100 input locations create approx. 10k routes.


lls = []

def read_locations():
  # read in location file to create routes
  with open(sys.argv[1] + "_locations.txt","r") as f:
    lls = list(f.read().splitlines()) #splitlines removes the \n
  f.close()
 
  return lls

def create_routes(lls): 	
  for c in range(0, len(lls)):
    for i in range(1, len(lls)):
      from_ll = re.split(',',str(lls[c]))
      to_ll = re.split(',',str(lls[i]))
      if sys.argv[3:] and sys.argv[4:]:
        print '-j \'' + json.dumps({'costing': sys.argv[2], 'locations': [{'lat': from_ll[0], 'lon': from_ll[1]}, {'lat': to_ll[0], 'lon': to_ll[1]}], 'date_time': {'type': int(sys.argv[3]), 'value': sys.argv[4]}}, sort_keys=True, separators=(',',':'))
      else:
        print '-j \'' + json.dumps({'costing': sys.argv[2], 'locations': [{'lat': from_ll[0], 'lon': from_ll[1]}, {'lat': to_ll[0], 'lon': to_ll[1]}]}, sort_keys=True, separators=(',',':'))
      

lls = read_locations()
create_routes(lls)

