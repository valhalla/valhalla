#!/usr/bin/env python
import sys, errno

#srtmgl1 is sampled at 1 arc second but the tiles are offset so that the value at a pixel is centered
#over .5 arc seconds. so we apply that here by basically adding and subtracting this (1.0/3600.0)*.5
half_arc_sec = (1.0/3600.0)*.5

for x in range(0, 360):
	for y in range(0, 180):
		try:
			tile_name = '%s%02d%s%03d.hgt' % ('S' if y < 90 else 'N', abs(y - 90), 'W' if x < 180 else 'E', abs(x - 180))
			print (x - 180) - half_arc_sec, (y - 90) - half_arc_sec, (x - 179) + half_arc_sec, (y - 89) + half_arc_sec, 'tiles/' + tile_name[0:3] + '/' + tile_name 
		except IOError as e:
			sys.exit(0)
