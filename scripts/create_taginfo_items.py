#!/usr/bin/env python2
import json
from sys import argv

keys = argv[1].split(argv[2])
values = argv[3].split(argv[4])

items = []
for key in keys:
  for value in values:
    items.append({'key':key,'value':value,'object_types':argv[5].split(','),'description':argv[6]})

print json.dumps(items)
