# Valhalla Live Traffic Builder

This repo is used for generating the live traffic CSV that will be ingested by Valhalla for real-time traffic updates.


## Setup

#### Install
```bash
npm install
```

#### Credentails

Update the `.env` with the API key listed in 1pass

```
INRIX_APP_ID
INRIX_HASH_TOKEN
```


#### Data
You will need a few input files that have been generated from Valhalla
```bash
./input/traffic_tile_offset.csv
./input/ways_to_edges.db
./input/xds_segments.db
```
These should be available in S3, and will need to be download to the `./input` directory before running the build scripts.

Additionally you will need the respective `traffic.tar` to update

```bash
mkdir -p input
s5cmd cp --concurrency 256 s3://io.radar.valhalla/2025-08-10/traffic_tile_offset.csv ./input/traffic_tile_offset.csv
s5cmd cp --concurrency 256 s3://io.radar.valhalla/2025-08-10/ways_to_edges.db ./input/ways_to_edges.db
s5cmd cp --concurrency 256 s3://io.radar.valhalla/2025-08-10/xds_segments.db ./input/xds_segments.db


mkdir -p ../valhalla-server
[ -f ../valhalla-server/traffic.tar ] || s5cmd cp --concurrency 256 s3://io.radar.valhalla/2025-08-10/traffic.tar ../valhalla-server/traffic.tar
```


## Usage

Running the traffic builder will update the traffic.tar with Inrix Traffic with USA, Mexico, and Canada


```bash
npm run build-inrix-traffic -- --processes=12 --valhallaPath='./node_bindings.node' --trafficPath='../valhalla-server/traffic.tar' --inputPath='./input'
```

Valhalla Traffic Builder uses NAPI to call Valhalla code to update the traffic.tar. The repo already contains a `node_bindings.node` file checked into to it. If you wanted to make updates to the `node_bindings.node` you can rebuild following these [steps](../README_RADAR.md#2-valhalla-traffic-worker) and modify the path


If you want to test the traffic.tar, the fastest turn around to have "live traffic", is to run Valhalla locally, with Valhalla Traffic Builder actually updating the traffic.tar file. Then you can point your local dev server to Valhalla, and use the Superuser Route Compare and Superuser Inspect tool to inspect the edges. Note that the update is in place, ie once the edges are updated you **do not** need to restart Valhalla, the edges will be updated automatically.

If this is too tedious, if staging is pointed at a different S3 bucket than production you can upload your data to staging's Valhalla's S3 bucket traffic.tar and wait 5 minutes for that to be pulled down and inspect at https://dashboard.radar-staging.com/superuser/route/inspect

#### Inrix Data notes

Inrix is split using Microsoft quadkeys starting at zoom level 7. https://learn.microsoft.com/en-us/bingmaps/articles/bing-maps-tile-system?redirectedfrom=MSDN

https://labs.mapbox.com/what-the-tile/ this is an excellent visualization for quadkeys.

TODO: Investigate Inrix offsets.

Valhalla maintains a set of edges

OSM maintains a set of edges

Inrix maintains a set of edges


Inrix provides a conflation from Inrix -> OSM. And Valhalla can generate an OSM -> Valhalla edge conflation


Both Inrix and Valhalla edges have offsets from OSM, however in practice offsets don't affect travel time that much, and Inrix offsets don't tend to be well formed
