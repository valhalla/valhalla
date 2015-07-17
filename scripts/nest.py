#!/usr/bin/env python
import sys, errno

'''
what we do is make a hierarchy

360 | 3 | 120 | 3 | 40 | 5 | 8 | 2 | 4 | 2 | 2
    | x |     | x |    | x |   | x |   | x |
180 | 3 |  60 | 3 | 20 | 5 | 4 | 2 | 2 | 2 | 1

'''
dims = [ 2, 2, 5, 3, 3 ]

width = 2
height = 1
level = 1

def t(x, y, l):
	if l == 5:
		return 'tiles/' + str(x) + '/' + str(y) + '.tif'
	else:
		return 'tiles/' + str(x) + '/' + str(y) + '_' + str(l) + '.vrt'

for dim in dims:
	for x in range(0, width):
		for y in range(0, height):
			files = []
			for i in range(0, dim):
				for j in range(0, dim):
					files.append(t(x * dim + i, y * dim + j, level))
			#TODO: prepend metatile name
			files.insert(0, t(x, y, level - 1))
			try:
				print ' '.join(files)
			except IOError as e:
				sys.exit(0)
	level += 1
	width *= dim
	height *= dim
