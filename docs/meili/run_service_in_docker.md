# Running Meili service in Docker

Make sure you have `docker` and `wget` installed.

## Get Meili Docker Image


Get meili from DockerHub:
```sh
sudo docker pull valhalla/docker

# you can also build it manually, which takes much longer time
# read Dockerfile.dev in a directory called docker in the branch master
# sudo docker build -t valhalla/docker -f Dockerfile-ppa https://github.com/valhalla/docker.git#master
```

## Build Graph Tiles

We need a data directory, let's say `/data` (must be absolute path),
to serve as a shared folder between host and the docker container. We
will place OSM data, a configuration file and generated tiles there:

```sh
DATA_DIR=/data
mkdir -p "$DATA_DIR"
```

Search your city at
[Interline OSM Extracts](https://www.interline.io/osm/extracts) or your country at
[Geofabrik Downloads](http://download.geofabrik.de/index.html) and
download the OSM data to the data directory. Here we take Berlin as
example:

```sh
wget --directory-prefix "$DATA_DIR" https://s3.amazonaws.com/metro-extracts.mapzen.com/berlin_germany.osm.pbf
```

Build configuration:

```sh
sudo docker run -it \
     --volume  "$DATA_DIR":/data \
     --workdir /data \
     valhalla/docker \
     valhalla_build_config > /data/valhalla.json
```

Build tiles:

```sh
sudo docker run -it \
     --volume  "$DATA_DIR":/data \
     --workdir /data \
     valhalla/docker \
     valhalla_build_tiles --conf /data/valhalla.json berlin_germany.osm.pbf
```

This process takes time from a few minutes to hours, depending on the
OSM size. It creates tiles under `$DATA_DIR/valhalla`.


## Run Map Matching Service

Run the service:

```sh
sudo docker run -it \
     --volume "$DATA_DIR":/data \
     --publish 8002:8002 \
     valhalla/docker \
     valhalla_service /data/valhalla.json
```

Now the service is up and listening on `localhost:8002`. You may refer
to the
[service API](service_api.md)
for more information.


## Set up Demo

Open
http://valhalla.github.io/demos/map_matching/index-internal.html. Zoom to Berlin area. Simulate
a trace by clicking on the map. You will see the trace is map matched
immediately.
