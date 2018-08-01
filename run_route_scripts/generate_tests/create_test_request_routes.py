#!/usr/bin/env python
import sys
import math
import random
import json
import re

lls = []

def read_locations():
  # read in location file to create routes
  with open(sys.argv[1] + "_locations.txt","r") as f:
    lls = list(f.read().splitlines()) #splitlines removes the \n
  f.close()
 
  return lls

def create_routes(lls): 	
#need to loop to next line for 2nd latlng
  for c in range(0, len(lls)):
    for i in range(1, len(lls)):
      from_ll = re.split(',',str(lls[c]))
      to_ll = re.split(',',str(lls[i]))
      if sys.argv[3:] and sys.argv[4:]:
        print json.dumps({'costing': sys.argv[2], 'locations': [{'lat': from_ll[0], 'lon': from_ll[1]}, {'lat': to_ll[0], 'lon': to_ll[1]}], 'date_time': {'type': int(sys.argv[3]), 'value': sys.argv[4]}}, sort_keys=True, separators=(',',':'))
      else:
        print json.dumps({'costing': sys.argv[2], 'locations': [{'lat': from_ll[0], 'lon': from_ll[1]}, {'lat': to_ll[0], 'lon': to_ll[1]}]}, sort_keys=True, separators=(',',':'))
      

# use those 100 locations to generate 10K routes
lls = read_locations()
create_routes(lls)

