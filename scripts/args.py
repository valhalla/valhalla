#!/usr/bin/env python
import sys, errno

for x in range(0, 360):
	for y in range(0, 180):
		try:
			print x - 180, y - 90, x - 179, y - 89, 'tiles/' + str(x) + '/' + str(y) + '.tif'
		except IOError as e:
			sys.exit(0)
