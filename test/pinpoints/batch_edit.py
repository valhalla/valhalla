#!/usr/bin/env python3

# first generate python bindings like so
# for f in  ../../proto/*.proto; do protoc -I=../../proto --python_out=. $f; done

import api_pb2
import os

# for each pbf
for root, dirs, files in os.walk(".", topdown=False):
  for name in files:
    if name.endswith('.pbf'):
      print('Reading %s' % os.path.join(root, name))
      # read the protobuf from disk
      with open(os.path.join(root, name), 'rb') as handle:
        pbf = handle.read()
      api = api_pb2.Api()
      api.ParseFromString(pbf)

      # MAKE YOUR CHANGES TO THE PROTOBUF HERE vvvvvvv
      #for r in api.trip.routes:
      #  for l in r.legs:
      #    for n in l.node:
      #      if n.edge is not None:
      #        n.edge.drive_on_left = not n.edge.drive_on_left
      #api.options.roundabout_exits = True
      #if not api.options.language:
      #  api.options.language = 'en-US'
      api.options.action -= 1
      # MAKE YOUR CHANGES TO THE PROTOBUF HERE ^^^^^^^

      # write the protobuf back to disk
      with open(os.path.join(root, name), 'wb') as handle:
        handle.write(api.SerializeToString())
      print('Wrote %s' % os.path.join(root, name))
