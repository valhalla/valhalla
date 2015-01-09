#!/usr/bin/env python

import sys
import BaseHTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler
from cgi import parse_qs, escape

'''
sample url looks like this:

http://localhost:8080/car/viaroute?z=17&output=json&instructions=true&loc=40.657912,-73.914450&loc=40.040501,-76.306271

http://localhost:8080/car/locate?loc=40.657912,-73.914450

http://localhost:8080/car/nearest?loc=40.657912,-73.914450
'''

#mapping mapzen osrm profiles to our internal costing algorithms
costing_methods = {'car': 'auto', 'bicycle': 'bicycle', 'foot': 'pedestrian'}
#mapping actions to internal methods to call with the input
#TODO: these will be methods to boost python bindings into the tyr library
actions = {'locate': str, 'nearest': str, 'viaroute': str}

#custom handler for getting routes
class TyrHandler(SimpleHTTPRequestHandler):

  #get a new request
  def __init__(self,req,client_addr,server):
    SimpleHTTPRequestHandler.__init__(self,req,client_addr,server)

  #parse the request because we dont get this for free!
  def parse_path(self):
    if len(self.path) > 1024:
      raise Exception('DOS someone else jerkface')
    #split the query from the path
    try:
      pos = self.path.find('?')
      split = self.path[1:pos].split('/')
      split.append(self.path[pos + 1:])
      if len(split) != 3:
        raise
    except:
      raise Exception('Try a url with 3 components: profile/action?querystring')
    #path has the costing method and action in it
    action = actions[escape(split[1])]
    if action is None:
      raise Exception('Try a valid action: ' + str([k for k in actions]))
    costing_method = costing_methods[escape(split[0])]
    if costing_method is None:
      raise Exception('Try a valid costing method: ' + str([k for k in costing_methods]))
    #parse the bits of the query out
    params = parse_qs(split[2])
    #throw costing method in
    params['costing_method'] = costing_method
    #throw in path to config file
    params['config'] = 'conf/pbf2graph.json'
    #do the action
    #just send the dict over to c++ and use it directly
    result = action(params)
    #hand it back
    return result

  #send a success
  def succeed(self, response):
    self.send_response(200)

    #set some basic info
    self.send_header('Access-Control-Allow-Origin','*')
    self.send_header('Content-type', 'application/json;charset=utf-8')
    self.send_header('Content-length', len(response))
    self.end_headers()

    #hand it back
    self.wfile.write(response.encode('utf-8'))
    self.wfile.flush()

  #send a fail
  def fail(self, error):
    self.send_response(400)

    #set some basic info
    self.send_header('Access-Control-Allow-Origin','*')
    self.send_header('Content-type', 'application/json;charset=utf-8')
    self.send_header('Content-length', len(error))
    self.end_headers()

    #hand it back
    self.wfile.write(str(error).encode('utf-8'))
    self.wfile.flush()

  #handle the request
  def do_GET(self):

    #get out the bits we care about
    try:
      request = self.parse_path()
      #TODO: run the route
      self.succeed(request)
    except Exception as e:
      self.fail(str(e))

#go off an wait for connections
if __name__ == '__main__':
  #check for a port
  if sys.argv[1:]:
    port = int(sys.argv[1])
  else:
    port = 8002

  try:
    #setup the server
    server_address = ('127.0.0.1', port)
    TyrHandler.protocol_version = 'HTTP/1.1'
    httpd = BaseHTTPServer.HTTPServer(server_address, TyrHandler)

    #open the socket and serve
    sa = httpd.socket.getsockname()
    print 'Serving HTTP on', sa[0], 'port', sa[1], '...'

    httpd.serve_forever()
  except Exception as e:
    print str(e)
    sys.exit(1)
