#!/usr/bin/env python
import sys
import math
import random
import json

### Enter a location in an urban area and then generate a set random locations nearby. 
### Continue to use a similar process to generate a smaller number near surrounding urban areas
### until you get the number of locations you need. Next, these locations are used to generate approx. 10K routes

## input radius in meters
RADIUS = int(sys.argv[4])
RADIUS_IN_DEGREES = RADIUS / 110567.0
inputloc = (float(sys.argv[1]), float(sys.argv[2]))
randlocs = []

def get_random_locs(inputloc_x, inputloc_y):
  for i in range(0,int(sys.argv[3])):  # Choose number of lat/lngs to be generated
    # creates a more uniform range around the center point 
    a = float(random.uniform(0.0, 1.0))
    b = float(random.uniform(0.0, 1.0))
    r = RADIUS_IN_DEGREES * math.sqrt(a)
    c = 2 * math.pi * b
    x = r * math.cos(c) 
    y = r * math.sin(c)
    xLat  = x + inputloc_x
    yLong = y + inputloc_y
    randlocs.append([round(xLat, 5),round(yLong, 5)]);
  return randlocs


get_random_locs(inputloc[0], inputloc[1])
print (randlocs)

