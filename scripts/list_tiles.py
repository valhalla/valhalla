#!/usr/bin/env python3
#Author: Greg Knisely
#TileHierarchy and Tiles logic based on 
#https://github.com/valhalla/valhalla/blob/master/src/baldr/tilehierarchy.cc and
#https://github.com/valhalla/valhalla/blob/master/src/baldr/graphtile.cc

import sys
import math
import getopt
from distutils.util import strtobool

#world bb
minx_ = -180
miny_ = -90
maxx_ = 180
maxy_ = 90

#global vars
boundingbox = None
suffix = None

class BoundingBox(object):

  def __init__(self, min_x, min_y, max_x, max_y):
     self.minx = min_x
     self.miny = min_y
     self.maxx = max_x
     self.maxy = max_y

class TileHierarchy(object):

  def __init__(self):
    self.levels = {}
    # local
    self.levels[2] = Tiles(BoundingBox(minx_,miny_,maxx_,maxy_),.25)
    # arterial
    self.levels[1] = Tiles(BoundingBox(minx_,miny_,maxx_,maxy_),1)
    # highway
    self.levels[0] = Tiles(BoundingBox(minx_,miny_,maxx_,maxy_),4)

class Tiles(object):

  def __init__(self, bbox, size):
     self.bbox = bbox
     self.tilesize = size

     self.ncolumns = int(math.ceil((self.bbox.maxx - self.bbox.minx) / self.tilesize))
     self.nrows = int(math.ceil((self.bbox.maxy - self.bbox.miny) / self.tilesize))
     self.max_tile_id = ((self.ncolumns * self.nrows) - 1)

  def Row(self, y):
    #Return -1 if outside the tile system bounds
    if (y < self.bbox.miny or y > self.bbox.maxy):
      return -1

    #If equal to the max y return the largest row
    if (y == self.bbox.maxy):
      return self.nrows - 1
    else:
      return int((y - self.bbox.miny) / self.tilesize)

  def Col(self, x):
    #Return -1 if outside the tile system bounds
    if (x < self.bbox.minx or x > self.bbox.maxx):
      return -1

    #If equal to the max x return the largest column
    if (x == self.bbox.maxx):
      return self.ncolumns - 1
    else:
      col = (x - self.bbox.minx) / self.tilesize
      return int(col) if (col >= 0.0) else int(col - 1)

  def Digits(self, number):
    digits = 1 if (number < 0) else 0
    while number:
       number /= 10
       digits += 1
    return digits

  # get the File based on tile_id and level
  def GetFile(self, tile_id, level):

    max_length = self.Digits(self.max_tile_id)

    remainder = max_length % 3
    if remainder:
       max_length += 3 - remainder

    #if it starts with a zero the pow trick doesn't work
    if level == 0:
      file_suffix = '{:,}'.format(int(pow(10, max_length)) + tile_id).replace(',', '/')
      file_suffix += "."
      file_suffix += suffix
      file_suffix = "0" + file_suffix[1:]
      return file_suffix

    #it was something else
    file_suffix = '{:,}'.format(level * int(pow(10, max_length)) + tile_id).replace(',', '/')
    file_suffix += "."
    file_suffix += suffix
    return file_suffix

def check_args(argv):

   global boundingbox
   global suffix
   try:
      opts, args = getopt.getopt(sys.argv[1:], "h:b:s:", ["help=", "bbox=", "suffix="])
   except getopt.GetoptError:
      print('tiles.py -b lower_left_lng_lat, upper_right_lng_lat -s file_suffix')
      print('tiles.py -b -74.251961,40.512764,-73.755405,40.903125 -s json')
      sys.exit(2)
   for opt, arg in opts:
      if opt in ("-h", "--help"):
         print('tiles.py -b lower_left_lng_lat, upper_right_lng_lat -s file_suffix')
         print('tiles.py -b -74.251961,40.512764,-73.755405,40.903125 -s json')
         sys.exit()
      elif opt in ("-b", "--bbox"):
         boundingbox = arg
      elif opt in ("-s", "--suffix"):
         suffix = arg

   if (boundingbox == None or suffix == None):
      print('tiles.py -b lower_left_lng_lat, upper_right_lng_lat -s file_suffix')
      print('tiles.py -b -74.251961,40.512764,-73.755405,40.903125 -s json')
      sys.exit()

#this is the entry point to the program
if __name__ == "__main__":
   
  check_args(sys.argv[1:])

  # these are the tiles that should exist in s3
  tile_hierarchy = TileHierarchy()
  if boundingbox:
    bbox = [ float(i) for i in boundingbox.split(',')]

    bounding_boxes = []
    #check our bb and make sure it does not cross 180/-180, if it does
    #split it into two bounding boxes.

    if bbox[0] >= bbox[2]:
      bbox[0] = bbox[0] - 360

    if bbox[0] < minx_ and bbox[2] > minx_:
      # Create 2 bounding boxes
      range = maxx_ - minx_
      bounding_boxes.append(BoundingBox(minx_, bbox[1], bbox[2], bbox[3]))
      bounding_boxes.append(BoundingBox(bbox[0] + range, bbox[1], maxx_, bbox[3]))
    elif bbox[0] < maxx_ and bbox[2] > maxx_:
      # Create 2 bounding boxes
      range = maxx_ - minx_
      bounding_boxes.append(BoundingBox(bbox[0], bbox[1], maxx_, bbox[3]))
      bounding_boxes.append(BoundingBox(minx_, bbox[1], bbox[2] - range, bbox[3]))
    else:
      bounding_boxes.append(BoundingBox(bbox[0], bbox[1], bbox[2], bbox[3]))

    while (len(bounding_boxes) != 0):
      b_box = bounding_boxes.pop(0)
      for level, tiles in tile_hierarchy.levels.items():
        mincol = tiles.Col(b_box.minx)
        i = tiles.Row(b_box.miny)
        while i <= tiles.Row(b_box.maxy):
          tile_id = (i * tiles.ncolumns) + mincol
          j = mincol
          while j <= tiles.Col(b_box.maxx):
            file_name = tiles.GetFile(tile_id,level)
            print(file_name)
            tile_id += 1
            j += 1
          i += 1


