#!/usr/bin/env python
import sys
import math
import random
import json

RADIUS = int(sys.argv[4])
RADIUS_IN_DEGREES = RADIUS / 111300.0
inputloc = (float(sys.argv[1]), float(sys.argv[2]))
randlocs = []

def get_random_locs(inputloc_x, inputloc_y):
  for i in range(0,int(sys.argv[3])):  #Choose number of latlon to be generated
    a = float(random.uniform(0.0,1.0))
    b = float(random.uniform(0.0,1.0))
    r = RADIUS_IN_DEGREES * math.sqrt(a)
    c = 2 * math.pi * b
    x = r * math.cos(c) 
    y = r * math.sin(c)
    xLat  = x + inputloc_x
    yLong = y + inputloc_y
    randlocs.append([round(xLat, 5), round(yLong, 5)]);
  return randlocs

# enter a location in a city and then generate a set of 20 or so random locations nearby. Then a similar process 
# to generate a smaller number near Anahiem, etc. until you get 100 locations. But then use those 100 locations to generate 10K routes
get_random_locs(inputloc[0], inputloc[1])
print (randlocs)

