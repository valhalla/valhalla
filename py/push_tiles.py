#!/usr/bin/env python
from __future__ import print_function
from datetime import datetime
import tarfile
import hashlib
import sys
import os
import boto

#tar a directory
def targzip(dst_file, src_dir):
  with tarfile.open(dst_file, mode = 'w:gz', compresslevel = 9) as tgz:
    tgz.add(src_dir, arcname=os.path.basename(src_dir))

#do some maintainance on s3, including pushing the new data up there
def push_to_s3(file_name):
  #check what we have in s3
  s3 = boto.connect_s3()
  bucket = s3.get_bucket('mapzen.valhalla')
  keys = sorted([ k.key for k in bucket.get_all_keys() ])
  keys.reverse()

  #remove it if its there
  if tgz_file in keys:
    bucket.delete_key(file_name)
    keys.remove(file_name)

  #lets keep it to a managable amount of previous ones
  while len(keys) > 50:
    key = keys.pop()
    bucket.delete_key(key)

  #get the size of the file and the md5sum
  md5 = hashlib.md5()
  size = 0
  with open(file_name, 'rb') as f:
    while True:
      buf = f.read(524288)
      if not buf:
        break
      size += len(buf)
      md5.update(buf)

  #put it up there
  key = bucket.new_key(file_name)
  key.size = size
  key.md5 = md5.hexdigest()
  with open(file_name) as f:
    key.send_file(f)
  if bucket.get_key(file_name) is None:
    raise 'Failed to push file to s3'


#get all the instances of a layer and run some recipes on each
def update_instances(stack, layer, recipes):
  #connect to opsworks
  opsworks = boto.connect_opsworks()

  #grab all the instances for that layer
  instances = opsworks.describe_instances(layer_id=layer)

  #TODO: make this more robust of a heuristic
  #if we dont have enough machines to feel safe we give up
  instances = [ instance for instance in instances['Instances'] if instance.get('Status') == 'online' ]
  if len(instances) < 3:
    raise 'Not enough instances in layer to risk the update'

  #for each one
  for instance in instances:
    print(instance)
    for recipe in recipes:
      #deploy task or something..
      opsworks.create_deployment(stack,
        '{"Name": "execute_recipes", "Args": {"recipes": ["%s"]}}' % recipe,
        instance_ids=[instance['InstanceId']])

#entry point for script
if __name__ == "__main__":
  if len(sys.argv) < 2:
    print('Wrong arguments', file=sys.stderr)
    sys.exit(1)

  #zip up the tiles
  tgz_file = datetime.utcnow().strftime("%Y_%m_%d-%H_%M_%S.tgz")
  targzip(tgz_file, sys.argv[1])

  #push them to s3
  push_to_s3(tgz_file)

  #update the service instances
  update_instances('978e7e69-0c63-46da-9e12-39a25a1f6078', 'c39b1588-3824-464e-9fbc-99d9882e39cc', ['valhalla::noop_test'])
