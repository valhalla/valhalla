#!/usr/bin/env python
import sys, errno

x0 = -180.0
while x0 < 180:
	y0 = -90
	while y0 < 90:
		if x0 < 0:
			d = 'tiles/w' + str(abs(x0))
		else:
			d = 'tiles/e' + str(abs(x0))
		if y0 < 0:
			f = d + '/s' + str(abs(y0)) + '.tif'
		else:
			f = d + '/n' + str(abs(y0)) + '.tif'
		try:
			print x0, y0, x0 + .25, y0 + .25, f
		except IOError as e:
			sys.exit(0)
		y0 += .25
	x0 += .25 
