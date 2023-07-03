#!/usr/bin/env python3
import sys

'''
what we do is make a hierarchy

360 | 3 | 120 | 3 | 40 | 5 | 8 | 2 | 4 | 2 | 2
    | x |     | x |    | x |   | x |   | x |
180 | 3 |  60 | 3 | 20 | 5 | 4 | 2 | 2 | 2 | 1

'''
dims = [2, 2, 5, 3, 3]

width = 2
height = 1
level = 1


# flake8: noqa: E741
def t(x, y, l, d):
    if l == 5:
        return ('./' if x == d else '../' + str(x) + '/') + str(y) + '.tif'
    else:
        return ('./' if x == d else '../' + str(x) + '/') + str(y) + '_' + str(l) + '.vrt'


for dim in dims:
    for x in range(0, width):
        for y in range(0, height):
            cmd = []
            for i in range(0, dim):
                for j in range(0, dim):
                    cmd.append(t(x * dim + i, y * dim + j, level, x))
            cmd.insert(0, 'gdalbuildvrt ./' + str(y) + '_' + str(level - 1) + '.vrt')
            cmd.insert(0, 'cd tiles/%s;' % x)
            try:
                print(' '.join(cmd))
            except IOError:
                sys.exit(0)
    level += 1
    width *= dim
    height *= dim
