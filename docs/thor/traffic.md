# Traffic Influenced Routing 

Valhalla allows traffic speeds to be set on OpenStreetMap (OSM) ways. These ways are then correlated with Valhalla graph edges to generate a txt file which maps OSM way id to Valhalla edge id. This file can be used to create a csv file of edge ids and their associated traffic speeds. The csv file can then be loaded by Valhalla and the edges in each tile will be updated with the speed to produce traffic-influenced routes.

## Generating Edge IDs

To generate the OSM way id to Valhalla edge id file, run
```
valhalla_ways_to_edges  --config valhalla.json
```
This will output a file called `way_edges.txt` in the tile directory folder (default `valhalla_tiles/`) with the format
```csv
<osm_way_id>, <direction>, <edge_id>, <direction>, <edge_id>, ...
```
Where `direction` is 1 for forward and 0 for reverse.

For example:
```
10671807,0,2623330114,1,2656884546,1,2824656706,0,2858211138
10632778,1,1717360442,0,1784469306
5470910,1,1046271802,0,1079826234
5457887,0,408737594,1,442292026,0,542955322,1,576509754,1,1918687034,0,2891765570
```

## Creating Speed CSV
To update the speeds for particular edges, create a CSV file with the following format:
```
<edge_id>, <speed in kmh> 
```
Make sure to use the Valhalla edge ids and not the OSM way ids. Put the CSV file into a subdirectory called `speeds/`.

## Setting Speeds in Tiles
First, make sure the config file (default `valhalla.json`) does not have the field `tile_extract` set. See [#2006](https://github.com/valhalla/valhalla/issues/2006) 

Apply the speed csv file to the tiles by running
```
valhalla_add_predicted_traffic -c valhalla.json -t speeds/
```
Routing should now take the updated speeds into account.
