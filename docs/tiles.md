# Tile Specifications

## Hierarchies/Levels
Tiles are split up into three levels or hierarchies.  Hierarchy 0 contains edges pertaining to roads that are considered highway (motorway, trunk, and primary) roads and are stored in 4 degree tiles.  Hierarchy 1 contains roads that are at a arterial level (secondary and tertiary) and are saved in 1 degree tiles.  Finally, Hierarchy 2 contains roads that are considered at a local level (unclassified, residential, and service or other).  These tiles are saved in .25 degree tiles.

So in python, the levels are defined as:<br/>
`valhalla_tiles = [{'level': 2, 'size': 0.25}, {'level': 1, 'size': 1.0}, {'level': 0, 'size': 4.0}]`

### The World at Level 0

The following image shows the world at level 0.  Using a world bounding box (-180, -90, 180, 90) the world is split up into 4 degree tiles.  The rows and columns start from the bottom left and increase to the top right.  Tiles are row ordered increasing from west to east.

![Level 0](images/world_level0.png)
Image generated using http://geojson.io

Using a bounding box for Germany, Pennsylvania, and NYC we can show how the regions would be split up into the 3 levels.  Level 0 is colored in light blue.  Level 1 is light green and level 2 is light red.

### Germany
![Germany](images/germany.png)
Image generated using http://geojson.io

### Pennsylvania
![Pennsylvania](images/pennsylvania.png)
Image generated using http://geojson.io

### NYC
![NYC](images/nyc.png)
Image generated using http://geojson.io

# Sample Tile Code
Below are some sample functions to help you obtain latitude and longitude coordinates, levels, tile ids, and list of tiles that intersect a bounding box.
```
#!/usr/bin/env python

valhalla_tiles = [{'level': 2, 'size': 0.25}, {'level': 1, 'size': 1.0}, {'level': 0, 'size': 4.0}]
LEVEL_BITS = 3
TILE_INDEX_BITS = 22
ID_INDEX_BITS = 21
LEVEL_MASK = (2**LEVEL_BITS) - 1
TILE_INDEX_MASK = (2**TILE_INDEX_BITS) - 1
ID_INDEX_MASK = (2**ID_INDEX_BITS) - 1
INVALID_ID = (ID_INDEX_MASK << (TILE_INDEX_BITS + LEVEL_BITS)) | (TILE_INDEX_MASK << LEVEL_BITS) | LEVEL_MASK

def get_tile_level(id):
  return id & LEVEL_MASK

def get_tile_index(id):
  return (id >> LEVEL_BITS) & TILE_INDEX_MASK

def get_index(id):
  return (id >> (LEVEL_BITS + TILE_INDEX_BITS)) & ID_INDEX_MASK

def tiles_for_bounding_box(left, bottom, right, top):
  #if this is crossing the anti meridian split it up and combine
  if left > right:
    east = tiles_for_bounding_box(left, bottom, 180.0, top)
    west = tiles_for_bounding_box(-180.0, bottom, right, top)
    return east + west
  #move these so we can compute percentages
  left += 180
  right += 180
  bottom += 90
  top += 90
  tiles = []
  #for each size of tile
  for tile_set in valhalla_tiles:
    #for each column
    for x in range(int(left/tile_set['size']), int(right/tile_set['size']) + 1):
      #for each row
      for y in range(int(bottom/tile_set['size']), int(top/tile_set['size']) + 1):
        #give back the level and the tile index
        tiles.append((tile_set['level'], int(y * (360.0/tile_set['size']) + x)))
  return tiles

def get_tile_id(tile_level, lat, lon):
  level = filter(lambda x: x['level'] == tile_level, valhalla_tiles)[0]
  width = int(360 / level['size'])
  return int((lat + 90) / level['size']) * width + int((lon + 180 ) / level['size'])

def get_ll(id):
  tile_level = get_tile_level(id)
  tile_index = get_tile_index(id)
  level = filter(lambda x: x['level'] == tile_level, valhalla_tiles)[0]
  width = int(360 / level['size'])
  height = int(180 / level['size'])
  return int(tile_index / width) * level['size'] - 90, (tile_index % width) * level['size'] - 180

```
## Get the Level or Hierarchy
`get_tile_level(73160266)` returns a level of 2.  73160266 is a Valhalla Graphid.<br/>
`get_tile_level(142438865769)` returns a level of 1.  142438865769 is an Open Traffic Segment id.<br/> 
## Get the Latitude and Longitude from an ID
`get_ll(73160266)` returns the bottom left corner of the level 2 tile.  (41.25, -73.75)<br/>
`get_ll(142438865769)` returns the bottom left corner of the level 1 tile.  (14.0, 121.0)<br/>
## Get the Tile ID from a Latitude, Longitude, and Level
`get_tile_id(0, 14.601879, 120.972545)`  2415	Tile would be on disk with the directory structure of `/0/002/415.gph`<br/>
`get_tile_id(1, 14.601879, 120.972545)`  37740	Tile would be on disk with the directory structure of `/1/037/740.gph`<br/>
`get_tile_id(2, 41.413203, -73.623787)`  756425 Tile would be on disk with the directory structure of `/2/000/756/425.gph`<br/>
## Get Tile ID from an ID
`get_tile_index(73160266)`  Returns the tile id 756425.<br/>
`get_tile_index(142438865769)`  Returns the tile id 37741.<br/>
## Get Tiles That Intersect a Bounding Box
`tiles_for_bounding_box(-74.251961,40.512764,-73.755405,40.903125)`  Returns a list of tiles for the NYC bounding box at each level.  
`[(2, 752102), (2, 753542), (2, 752103), (2, 753543), (2, 752104), (2, 753544), (1, 46905), (1, 46906), (0, 2906)]`  

