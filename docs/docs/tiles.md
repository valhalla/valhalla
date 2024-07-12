# Tile Specifications

## Hierarchies/Levels
Tiles are split up into 3 levels or hierarchies:<br/>
`Hierarchy 0:` contains edges pertaining to roads that are considered highway (motorway, trunk, and primary) roads and are stored in 4 degree tiles.<br/>
`Hierarchy 1:` contains roads that are at a arterial level (secondary and tertiary) and are saved in 1 degree tiles.<br/>
`Hierarchy 2:` contains roads that are considered at a local level (unclassified, residential, and service or other) and are saved in .25 degree tiles.<br/>

So in python, the levels are defined as:<br/>
`valhalla_tiles = [{'level': 2, 'size': 0.25}, {'level': 1, 'size': 1.0}, {'level': 0, 'size': 4.0}]`

At level 0, the world (-180, -90, 180, 90) is split up into 4 degree tiles. The rows and columns start from the bottom left and increase to the top right. Tiles are row ordered increasing from west to east.

Example: Using a bounding box for Pennsylvania we can show how the regions would be split up into the 3 levels. Level 0 is colored in light blue. Level 1 is light green and level 2 is light red.

![Pennsylvania](images/pennsylvania.png)
Image generated using http://geojson.io

# Sample Tile Code

Below are some sample functions to help you obtain latitude and longitude coordinates, levels, tile ids, and list of tiles that intersect a bounding box.

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
