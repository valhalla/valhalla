#!/usr/bin/env python
from __future__ import print_function
import boto
import tarfile
import urllib2
import sys
import os

def download(url, file_name):
  request = urllib2.urlopen(url)
  with open(file_name, 'wb') as f:
    while True:
      buf = request.read(524288)
      if not buf:
        break
      f.write(buf)

#entry point for script
if __name__ == "__main__":
  if len(sys.argv) < 2:
    print('Wrong arguments', file=sys.stderr)
    sys.exit(1)
  tile_dir = sys.argv[1]
 
  #check what we have in s3
  s3 = boto.connect_s3()
  bucket = s3.get_bucket('mapzen.valhalla')
  keys = sorted([ k.key for k in bucket.get_all_keys() if k.key.startswith('dev/tiles_') if k.key.endswith('tgz') ])
  keys.reverse()

  #download it or fail
  if len(keys) == 0:
    print('No tiles!', file=sys.stderr)
    sys.exit(1)
  tgz_file = tile_dir + '/' + os.path.basename(keys[0])
  url = 'https://s3.amazonaws.com/mapzen.valhalla/' + keys[0]
  download(url, tgz_file)

  #unpack it
  with tarfile.open(tgz_file) as tar:
    tar.extractall(path = tile_dir)
  os.remove(tgz_file)
