# Running Meili service in Docker

Make sure you have `docker`, `git` and `wget` installed.

## Build Meili Docker Image

Clone meili and build it
```sh
git clone --depth=1 https://github.com/valhalla/meili.git
sudo docker build -t valhalla/meili meili/docker
```

## Build Graph Tiles

We need a data directory, let's say `/data` (must be absolute path),
to serve as a shared folder between host and the docker container. We
will place OSM data and generated tiles there:

```sh
DATA_DIR=/data
mkdir -p "$DATA_DIR"
```

Search your city at
[Mapzen metro extracts](https://mapzen.com/data/metro-extracts/) or
[Geofabrik Downloads](http://download.geofabrik.de/index.html) and
download the OSM data to the data directory. Here we take Berlin as
example:

```sh
wget --directory-prefix "$DATA_DIR" https://s3.amazonaws.com/metro-extracts.mapzen.com/berlin_germany.osm.pbf
```

Build tiles:

```sh
sudo docker run -it \
     --volume  "$DATA_DIR":/data \
     --workdir /data \
     valhalla/meili \
     valhalla_build_tiles --conf /source/conf/valhalla.json berlin_germany.osm.pbf
```

This process takes time from a few minutes to hours, depending on the
OSM size. It creates tiles under `$DATA_DIR/valhalla`.


## Run Meili Service

Run the service:

```sh
sudo docker run -it \
     --volume "$DATA_DIR":/data \
     --publish 8002:8002 \
     valhalla/meili \
     valhalla_map_match_service /source/conf/valhalla.json
```

Now the service is up and listening on `localhost:8002`. You may refer
to the
[service API](https://github.com/valhalla/meili/blob/master/docs/service_api.md)
for more information.
