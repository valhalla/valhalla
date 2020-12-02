#!/usr/bin/env python3
import argparse
import multiprocessing
import requests
import json
import os
import shutil
import datetime
import time

# generator for post bodies from the file
def get_post_bodies(filename):
  with open(filename, 'r') as f:
    line_number = 0
    for line in f:
      line_number += 1
      line = line[line.find('{'):]
      line = line[0:line.rfind('}') + 1]
      post_body = json.loads(line)
      post_body['id'] = str(line_number)
      yield post_body

# for persistant connections
def initialize(url_, output_dir_):
  global session
  session = requests.Session()
  global url
  url = url_
  global output_dir
  output_dir = output_dir_

# post a request
def make_request(post_format):
  # open a file to put the result
  post_body = post_format[0]
  output_file = os.path.join(output_dir, post_body['id'] + '.' + ('csv' if post_format[1] == 'csv' else 'json'))
  with open(output_file, 'w') as f:
    try:
      # make request
      start = time.time()
      response = session.post(url, json=post_body).json()
      stop = time.time()
      elapsed = stop - start

      # raw json
      if post_format[1] == 'raw':
          f.write('%s' % json.dumps(response, sort_keys=True, indent=2))

      # summary json
      elif post_format[1] == 'json':
        out = {'routes':[{'legs':[]}]}
        for leg in response['trip']['legs']:
          out['routes'][-1]['legs'].append({'maneuvers':[]})
          for man in leg['maneuvers']:
            out['routes'][-1]['legs'][-1]['maneuvers'].append({'length': man['length'], 'time': man['time'], 'instruction': man['instruction']})
        out['performance'] = {'response_time': elapsed}
        f.write('%s' % json.dumps(out, sort_keys=True, indent=2))

      # csv
      else:  
        f.write('length (meters), time (seconds), instruction' + os.linesep)
        for leg in response['trip']['legs']:
          for man in leg['maneuvers']:
            f.write('%d,%d,%s%s' % (man['length']*1000, man['time'], man['instruction'],os.linesep))
     
    except Exception as e:
      f.write('%s' % e)

if __name__ == "__main__":
  # parse some program arguments
  parser = argparse.ArgumentParser()
  parser.add_argument('--test-file', type=str, help='The file with the test requests', required=True)
  parser.add_argument('--url', type=str, help='The url to which you want to POST the request bodies', default='http://localhost:8002/route')
  parser.add_argument('--output-dir', type=str, help='The directory in which to place the result of each request')
  parser.add_argument('--concurrency', type=int, help='The number of processes to use to make requests', default=multiprocessing.cpu_count())
  parser.add_argument('--format', type=str, help='Supports csv and json output formats', default='csv')
  args = parser.parse_args()

  # make the output directory
  if args.output_dir is None:
    basename = os.path.basename(os.path.splitext(args.test_file)[0])
    datestr = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    args.output_dir = '_'.join([datestr, basename])
  if os.path.exists(args.output_dir):
    shutil.rmtree(args.output_dir)
  os.mkdir(args.output_dir)

  # make a worker pool to work on the requests
  work = [(body, args.format) for body in get_post_bodies(args.test_file)]
  with multiprocessing.Pool(initializer=initialize(args.url, args.output_dir), processes=args.concurrency) as pool:
    result = pool.map_async(make_request, work)

    # check progress
    print('Placing %d results in %s' % (len(work), args.output_dir))
    progress = 0
    increment = 5
    while not result.ready():
      result.wait(timeout=5)
      done = len([f for f in os.listdir(args.output_dir) if os.path.isfile(os.path.join(args.output_dir, f))])
      next_progress = int(done / len(work) * 100)
      if int(next_progress / increment) > progress:
        print('%d%%' % next_progress)
        progress = int(next_progress / increment)
    if progress != 100 / increment:
      print('100%')
