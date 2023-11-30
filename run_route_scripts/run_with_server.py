#!/usr/bin/env python3
import argparse
import multiprocessing
import requests
import json
import os
from pathlib import Path
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

def initialize(args_,response_count_):
  # for persistent connections
  global session
  session = requests.Session()
  # so each process knows the options provided
  global args
  args = args_
  # so each process can signal completing a request
  global response_count
  response_count = response_count_

# post a request
def make_request(post_body):
  # make request
  try:
    start = time.time()
    response = session.post(args.url, json=post_body, headers=args.headers)
    stop = time.time()
    elapsed = stop - start
    response = response.json()
    with response_count.get_lock():
      response_count.value += 1
  except Exception as e:
    print(e)

  # nothing to write
  if args.format == 'null':
    return

  # open a file to put the result
  output_file = os.path.join(args.output_dir, post_body['id'] + '.' + ('csv' if args.format == 'csv' else 'json'))
  with open(output_file, 'w') as f:
    try:
      # raw json
      if args.format == 'raw':
          f.write('%s' % json.dumps(response, sort_keys=True, indent=2))

      # summary json
      elif args.format == 'json':
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
            f.write('%d,%d,%s%s' % (man['length']*1000, man['time'], man['instruction'], os.linesep))
     
    except Exception as e:
      f.write('%s' % e)

if __name__ == "__main__":
  # parse some program arguments
  parser = argparse.ArgumentParser()
  parser.add_argument('--test-file', type=str, help='The file with the test requests', required=True)
  parser.add_argument('--url', type=str, help='The url to which you want to POST the request bodies', default='http://localhost:8002/route')
  parser.add_argument('--output-dir', type=str, help='The directory in which to place the result of each request')
  parser.add_argument('--concurrency', type=int, help='The number of processes to use to make requests', default=multiprocessing.cpu_count())
  parser.add_argument('--format', type=str, help='Supports csv, json, raw and null output formats', default='csv')
  parser.add_argument('--headers', type=str, help='Additional http headers to send with the requests. Follows the http header spec, eg. some-header-name: some-header-value', action='append', nargs='*', default=[])
  parsed_args = parser.parse_args()

  # make the output directory
  if parsed_args.output_dir is None:
    basename = os.path.basename(os.path.splitext(parsed_args.test_file)[0])
    datestr = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    parsed_args.output_dir = '_'.join([datestr, basename])
  if os.path.exists(parsed_args.output_dir):
    shutil.rmtree(parsed_args.output_dir)
  os.mkdir(parsed_args.output_dir)

  # setup http headers
  parsed_args.headers = { k: v for k, v in [ h.split(': ') for hs in parsed_args.headers for h in hs] }
  # track progress with a count of finished requests
  response_count = multiprocessing.Value('i', 0)
  # make a worker pool to work on the requests
  work = [body for body in get_post_bodies(parsed_args.test_file)]
  # Note: workers also call initialize for themselves
  with multiprocessing.Pool(initializer=initialize, initargs=(parsed_args,response_count), processes=parsed_args.concurrency) as pool:
    result = pool.map_async(make_request, work)

    # check progress
    if parsed_args.format != 'null':
      print('Placing %d results in %s' % (len(work), parsed_args.output_dir))
    progress = 0
    increment = 5
    while not result.ready():
      result.wait(timeout=5)      
      next_progress = int(response_count.value / len(work) * 100)
      if int(next_progress / increment) > progress:
        print('%d%%' % next_progress)
        progress = int(next_progress / increment)
    if progress != 100 / increment:
      print('100%')

  # print total duration
  if parsed_args.format == "json":
    output_dir_path = Path(parsed_args.output_dir)
    duration = 0
    for out_f in output_dir_path.iterdir():
      if out_f.is_file() and out_f.suffix == ".json":
        with open(out_f) as f:
          try:
              duration += json.load(f)["performance"]["response_time"]
          except json.decoder.JSONDecodeError:
              continue

    print(f"Requests took {duration} seconds")
