#!/usr/bin/env python

import sys
import StringIO
import json
import threading
from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
from SocketServer import ThreadingMixIn
from cgi import urlparse
import tyr_service

'''
sample url looks like this:

http://localhost:8002/viaroute?z=17&output=json&instructions=true&loc=40.657912,-73.914450&loc=40.040501,-76.306271&costing_method=auto
http://localhost:8002/locate?loc=40.657912,-73.914450
http://localhost:8002/nearest?loc=40.657912,-73.914450
'''

#mapping actions to internal methods to call with the input
#TODO: these will be methods to boost python bindings into the tyr library
actions = {'locate': tyr_service.LocateHandler, 'nearest': tyr_service.NearestHandler, 'viaroute': tyr_service.RouteHandler, 'route': tyr_service.CustomRouteHandler}
methods = ['auto', 'pedestrian', 'bicycle']

#enable threaded server
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
  pass

#custom handler for getting routes
class TyrHandler(BaseHTTPRequestHandler):

  def __init__(self, *args):
    BaseHTTPRequestHandler.__init__(self, *args)

  #parse the request because we dont get this for free!
  def handle_request(self):
    if len(self.path) > 1024:
      raise Exception('DOS someone else jerkface')

    #split the query from the path
    try:
      split = urlparse.urlsplit(self.path)
    except:
      raise Exception('Try a url with 2 components: action?querystring')
    #path has the costing method and action in it
    try:
      action = actions[split.path.split('/')[-1]]
    except:
      raise Exception('Try a valid action: ' + str([k for k in actions]))
    #get a dict and unexplode non-list entries
    params = urlparse.parse_qs(split.query)
    for k,v in params.iteritems():
      if len(v) == 1:
        params[k] = v[0]
    #save jsonp or not
    jsonp = params.get('jsonp', None)
    if params.has_key('json'):
      params = json.loads(params['json'])
      if jsonp is not None:
        params['jsonp'] = jsonp
    if params.has_key('costing_method') == False:
      raise Exception('Try a valid costing_method: ' + str(methods))
    #do the action
    #just send the json over to c++ and parse it there
    result = action(json.dumps(params, separators=(',', ':'))).Action()
    #hand it back
    return result, jsonp is not None 

  #send a success
  def succeed(self, response, jsonp):
    self.send_response(200)

    #set some basic info
    self.send_header('Access-Control-Allow-Origin','*')
    if jsonp:
      self.send_header('Content-type', 'text/plain;charset=utf-8')
    else:
      self.send_header('Content-type', 'application/json;charset=utf-8')
    self.send_header('Content-length', len(response))
    self.end_headers()

    #hand it back
    self.wfile.write(response)

  #send a fail
  def fail(self, error):
    self.send_response(400)

    #set some basic info
    self.send_header('Access-Control-Allow-Origin','*')
    self.send_header('Content-type', 'text/plain;charset=utf-8')
    self.send_header('Content-length', len(error))
    self.end_headers()

    #hand it back
    self.wfile.write(str(error))

  #handle the request
  def do_GET(self):
    #get out the bits we care about
    try:
      response, jsonp = self.handle_request()
      self.succeed(response, jsonp)
    except Exception as e:
      self.fail(str(e))

#go off and wait for connections
if __name__ == '__main__':
  #check for a config file
  conf = {}
  try:
    with open(sys.argv[1]) as f:
      conf = json.load(f)
    tyr_service.Configure(sys.argv[1])
    conf = conf['tyr']
  except Exception as e:
    sys.stderr.write('Problem with config file: {0}\n'.format(e)) 
    sys.exit(1)

  #setup the server
  server = (conf.get('listen_address', '0.0.0.0'), conf.get('port', 8002))
  TyrHandler.protocol_version = 'HTTP/1.0'
  httpd = ThreadedHTTPServer(server, TyrHandler)

  try:
    httpd.serve_forever()
  except KeyboardInterrupt:
    httpd.server_close()
