# Historical Traffic Builder

This repo contains the script used to generate the predicted speed CSVs used for ingesting the data into Valhalla.

See more details here: https://valhalla.github.io/valhalla/mjolnir/historical_traffic/


### Install

```bash
npm install
npm install -g node-gyp
```


### Downloading Input Data
```bash
# Databases
s5cmd cp --concurrency 256 s3://io.radar.valhalla/2025-08-10/ways_to_edges.db .
s5cmd cp --concurrency 256 s3://io.radar.valhalla/2025-08-10/xds_segments.db .

# All of historic traffic data
mkdir -p /data/inrix
s5cmd cp  --concurrency 256 "s3://io.radar.valhalla/inrix-historic-traffic/*" ./data/inrix

# or just New York State
s5cmd cp  --concurrency 256 "s3://io.radar.valhalla/inrix-historic-traffic/USA_NY_NAS672XD_2501.csv" ./data/inrix/USA_NY_NAS672XD_2501.csv
```


###  Run the script

Build the C++ bindings (or run the node version). C++ bindings are much faster
```bash
node-gyp configure
node-gyp build
```

To run the script:
```Bash
# Inrix
node build-inrix-historic-traffic.js

# Handle bug in historic traffic builder
cd traffic/0
mv 000/* . && rmdir 000
cd ../1
mv 000/* . && rmdir 000
```
This will start the script, and create up to CONCURRENT_PROCESSES to process the pbf files (each process handles one file at a time from the queue).

CSV outputs will be written to `./traffic`. These can be added to Valhalla following these [steps at adding historic traffic](../README_RADAR.md#3-valhalla-data-pipeline)
```
find traffic/ -type f | head -n 10
traffic/0/000/003/340.csv
traffic/0/000/003/427.csv
traffic/0/000/003/341.csv
traffic/0/000/003/337.csv
traffic/0/000/003/246.csv
traffic/0/000/003/338.csv
traffic/0/000/003/517.csv
traffic/0/000/003/607.csv
traffic/0/000/003/429.csv
traffic/0/000/003/428.csv
```

Each row of the CSV contains:
```
graph_id,free_flow_speed,constrained_speed,speed_profile (encoded)
```


#### Notes
- Inrix comes in 15 minute intervals instead of 5 minutes, I'm still going to encode in 5 minute intervals into DCT II because that might be what Valhalla expects
- A singular Inrix edge can map to multiple OSM edges which is annoying to say the least
- Inrix edges appear in the traffic csv (I also downloaded neighbourhouring states) but don't appear in the inrix edge->OSM edge/some OSM edges are missing. I think the inrix data exist but I wonder if the mapping is broken
- Inrix edges appear to always populate a forward (P) and backward (N) edge

- Routing inspect superuser tool is your friend
