#!/usr/bin/env python
from __future__ import print_function
import boto

#entry point for script
if __name__ == "__main__":
  #check what we have in s3
  s3 = boto.connect_s3()
  bucket = s3.get_bucket('mapzen.valhalla')
  keys = sorted([ k.key for k in bucket.get_all_keys() if k.key.startswith('tiles_') ])
  keys.reverse()
  print('https://s3.amazonaws.com/mapzen.valhalla/' + keys[0])
