# Traffic in Valhalla

Traffic data for routing engines can be divided into two different types:

- *Historical traffic* describes the typically observed speed on a road at a specific point in time. So e.g. the typical speed on a Monday morning at 8 o'clock.
- *Live traffic* describes the currently observed speed on a road according to the actual traffic situation.

Valhalla supports both types of traffic for all APIs, excluding multimodal costing. For more information about how to route with traffic data and time information, check the respective API documentation. In the following it will be explained how to integrate historical traffic data into the valhalla graph.

## Graph Association

### With OSM IDs

If the available traffic data is associated to OSM ways, these ways have to be mapped on valhallas internal graph ids to then add the traffic data to the routing graph. The valhalla tool `valhalla_ways_to_edges` can be used to generate a mapping from OSM way ids to the valhalla graph ids. An example for an `way_edges.txt` file created by the tool looks like this:
```
1175181586,1,112642252344
984719585,1,110964530744,1,112508034616,1,112843578936
```
The format of each row is `<osm_way_id>,[<direction: 0 | 1>, <graph_id>]`. Accordingly, the `osm_way_id` is mapped to multiple `graph_id`'s. An `graph_id` can be converted to the required string format according to the `to_string` function of the class `valhalla::baldr::GraphId`, whose implementation can be found in the valhalla repository. The result is a string of the following form `level/tile_id/id` (e.g. `1/47701/130`).

## Historical Traffic

Valhalla requires the historical speed information in following CSV file format:

```
edge_id,freeflow_speed,constrained_speed,historical_speeds
1/47701/130,50,40,AQ0AAAAAAA...
1/47701/131,50,40,AQ0AAAAAAA...
```

The first three columns are required to assign historical traffic to an edge, while the `historical_speeds` column is optional for providing more detailed speed information. The `edge_id` column represents the internal `graph_id` of ways in the valhalla graph. Check the previous section to see how your data can be mapped on these `graph_id`'s. The columns `freeflow_speed` represents the typical speed during night and `constrained_speed` is the typical speed during day (in km/h). 

The `historical_speeds` column contains more detailed information about how the traffic on a segment changes over the week. It requires the estimated speed (in km/h) of a road segment in 5 minute intervals covering a whole week. This leads to a total of 2016 speed values per road segment, which should start at Sunday 0:00. The `historical_speeds` column contains an DCT-II encoded version of the 2016 speed values. It can be obtained by using functions whose implementation can be found in the valhalla source code. First `compress_speed_buckets(const float* speeds)` can be called to obtain an array of coefficients. In a next step `encode_compressed_speeds(const int16_t* coefficients)` can be used to convert these to the string encoded version of the speed values, which can be put into the CSV-file.

Finally, the speed of each way has to be put into the correct place in a folder hierarchy. This folder hierarch should look similar to valhalla’s tile hierarchy. An example tile hierarchy could look like this:
```
.
|-- 0
|   `-- 003
|       `-- 015.gph
|-- 1
|   `-- 047
|       `-- 701.gph
```

The traffic folder structure, containing the traffic CSV files, should accordingly look like this:
```
.
|-- 0
|   `-- 003
|       `-- 015.csv
|-- 1
|   `-- 047
|       `-- 701.csv
```

If a way's information is found in `0/003/015.gph`, then its matching speed values have to be saved in `0/003/015.csv`. To find the path in the tile hierarchy to save a speed value in, the function `GraphTile::FileSuffix`can be used to obtain the path from a `graph_id`. Its implementation can be found in the valhalla source code.

In a last step, the traffic data has to be added to the routing graph. This can be done by using the `valhalla_add_predicted_traffic` tool. Its parameter `-t` can be used to hand over the folder which contains the traffic CSV files.