#Traffic in Valhalla#

Traffic data for routing engines can be divided into two different types:

- *Historical traffic* describes the typically observed speed on a road at a specific point in time. So e.g. the typical speed on a Monday morning at 8 o'clock.
- *Live traffic* describes the currently observed speed on a road according to the actual traffic situation.

Valhalla supports both types of traffic for all APIs, but there are some excluded use-cases. Traffic doesn't work when using multimodal costing. Furthermore it is currently not supported for arrive_by timestamps in the source_to_targets API. For more information of how to route with traffic data/time information, check the respective API documentation. In the following it will be explained how to integrate historical and live traffic data into valhalla.

#Historical Traffic#

To make use of historical traffic data in valhalla, the estimated speed (in km/h) of a road segment has to be given in 5 minute intervals covering a whole week. This leads to a total of 2016 speed values per road segment. The data could look like this

way_id, speed_1, speed_2, ..., speed_2016
92728294,30,32,...50
681494965,45,46,...,70

where way_id represents the id of an Openstreetmap way. The column speed_1 represents the estimated speed between 00:00 to 00:05 on Sunday, speed_2 the speed between 00:05 to 00:10 on Sunday and so forth. (TODO: CORRECT!? Sunday?!)

This data has to be converted to a format which can be used by valhalla and has to be structured properly in a folder hierarchy similar to the valhalla tile hierarchy. Valhalla requires the speed information in following CSV file format:

edge_id, freeflow_speed, constrained_speed, historical_speeds
1/47701/130,50,40,AQ0AAAAAAA...
1/47701/131,50,40,AQ0AAAAAAA...

This CSV file can be created by following steps:

1. The edge_id column represents the internal graph id of the way in the valhalla graph. The valhalla tool 'valhalla_ways_to_edges' will generate a mapping from OSM way ids to these valhalla internal edge ids and save it in text file. An example for an way_edges.txt file would be:

1175181586,1,112642252344
984719585,1,110964530744,1,112508034616,1,112843578936

The first number is the OSM way id. It is followed by one or multiple internal edge ids which consist of a direction (0 or 1) and the according id. We don't need the direction here, but the edge ids. The edge_ids can now be converted to graph_ids according to the valhalla::baldr::GraphId implementation in the valhalla repository.

2. Freeflow speed represents the typical speed during night and constrained flow speed is the typical speed during day. You might come up with your own way of calculating these values based on the 2016 individual speed values.

3. The historical speeds are an DCT-II encoded version of the 2016 speed values. It can be obtained by using functions whose implementation can be found in the valhalla source code. First 
std::array<int16_t, kCoefficientCount> compress_speed_buckets(const float* speeds)
can be called to obtain an array of coefficients. In a next step 
std::string encode_compressed_speeds(const int16_t* coefficients)
can be used to get the string encoded version, which can be put into the CSV-file.

Finally, the speed of the each way has to be put into the correct place in a folder hierarchy. This folder hierarch should look similar to valhalla’s tile hierarchy. An example tile hierarchy could look like this:
```
valhalla_tiles
└───0
│   │
│   └───003
│       │   015.gph
│   
└───1
    └───047
        │   701.gph
...
```

The regarding traffic structure for the traffic data should accordingly look like this:
```
valhalla_traffic_tiles
└───0
│   │
│   └───003
│       │   015.csv
│   
└───1
    └───047
        │   701.csv
...
```

If a way_id's information is found in '0/003/015.gph', then its matching speed values have to be saved in '0/003/015.csv'. To find the path in the tile hierarchy to save the speed values in the implementation GraphTile::FileSuffix (see valhalla repository) to convert the graph_id into its regarding folder structure.

In a last step, the traffic data has to be added to the routing graph. This can be done by using the valhalla_add_predicted_traffic tool. Its parameter '-t' can be used to hand over the folder which contains the traffic CSV files.

#Live Traffic#

There is currently no official documentation how to integrate live traffic into valhalla. A good point to start using live traffic in valhalla would be reading the article at https://gis-ops.com/de/traffic-in-valhalla/ and checking the code + discussion of a proof of conception implementation at https://github.com/alinmindroc/valhalla_traffic_poc. DISCLAIMER: These are sources created by the community. They might not be complete and covering all aspects of how to handle live traffic in valhalla.
