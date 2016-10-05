     ██▒   █▓ ▄▄▄       ██▓     ██░ ██  ▄▄▄       ██▓     ██▓    ▄▄▄      
    ▓██░   █▒▒████▄    ▓██▒    ▓██░ ██▒▒████▄    ▓██▒    ▓██▒   ▒████▄    
     ▓██  █▒░▒██  ▀█▄  ▒██░    ▒██▀▀██░▒██  ▀█▄  ▒██░    ▒██░   ▒██  ▀█▄  
      ▒██ █░░░██▄▄▄▄██ ▒██░    ░▓█ ░██ ░██▄▄▄▄██ ▒██░    ▒██░   ░██▄▄▄▄██ 
       ▒▀█░   ▓█   ▓██▒░██████▒░▓█▒░██▓ ▓█   ▓██▒░██████▒░██████▒▓█   ▓██▒
       ░ ▐░   ▒▒   ▓▒█░░ ▒░▓  ░ ▒ ░░▒░▒ ▒▒   ▓▒█░░ ▒░▓  ░░ ▒░▓  ░▒▒   ▓▒█░
       ░ ░░    ▒   ▒▒ ░░ ░ ▒  ░ ▒ ░▒░ ░  ▒   ▒▒ ░░ ░ ▒  ░░ ░ ▒  ░ ▒   ▒▒ ░
         ░░    ░   ▒     ░ ░    ░  ░░ ░  ░   ▒     ░ ░     ░ ░    ░   ▒   
          ░        ░  ░    ░  ░ ░  ░  ░      ░  ░    ░  ░    ░  ░     ░  ░
         ░                                                                    

Valhalla is an open source routing engine and accompanying libraries for use with Open Street Map data. This repository is comprised of command line tools meant to exercise various bits of the valhalla libraries.

Build Status
------------

[![Circle CI](https://circleci.com/gh/valhalla/tools.svg?style=svg)](https://circleci.com/gh/valhalla/tools)

Valhalla Overview
-----------------

The are several key features that we hope can differentiate the valhalla project from other route engines. They are:

- Open source software, on open source data with a very liberal license. Should allow for transparency in development, encourage contribution and community input and foster use in other projects.
- Tiled hierarchical data structure. Should allow users to have a small memory footprint on memory constrained devices, enable offline routing, provide a means for regional extracts and partial updates.
- Runtime costing of edges and vertices within the graph via a plugin architecture. Should allow for customizable and alternate routes.
- C++ based API. Should allow for cross compilation of the various pieces to enable routing on offline portable devices.
- A plugin based narrative and maneuver generation architecture. Should allow for generation that is customized either to the administrative area or to the target locale.
- Multi-modal and time-based routes. Should allow for mixing auto, pedestrian, bike and public transportation in the same route or setting a time by which one must arrive at a location.

The valhalla organization is comprised of several repositories each responsible for a different function. The layout of the various projects is as follows:

- [Midgard](https://github.com/valhalla/midgard) - Basic geographic and geometric algorithms for use in the various other projects
- [Baldr](https://github.com/valhalla/baldr) - The base data structures for accessing and caching tiled route data. Depends on `midgard`
- [Sif](https://github.com/valhalla/sif) - Library used in costing of graph nodes and edges. This can be used as input to `loki` and `thor`. Depends on `midgard` and `baldr`
- [Skadi](https://github.com/valhalla/skadi) - Library and service for accessing elevation data. This can be used as input to `mjolnir` or as a standalone service. Depends on `midgard` and `baldr`
- [Mjolnir](https://github.com/valhalla/mjolnir) - Tools for turning open data into graph tiles. Depends on `midgard`, `baldr` and `skadi`
- [Loki](https://github.com/valhalla/loki) - Library used to search graph tiles and correlate input locations to an entity within a tile. This correlated entity (edge or vertex) can be used as input to `thor`. Depends on `midgard`, `baldr` and `sif`
- [Thor](https://github.com/valhalla/thor) - Library used to generate a path through the graph tile hierarchy. This path can be used as input to `odin`. Depends on `midgard`, `baldr`, `sif` and `odin`
- [Odin](https://github.com/valhalla/odin) - Library used to generate maneuvers and narrative based on a path. This set of directions information can be used as input to `tyr`. Depends on `midgard` and `baldr`
- [Tyr](https://github.com/valhalla/tyr) - Service used to handle http requests for a route communicating with all of the other valhalla APIs. The service will format output from `odin` and support json (and eventually protocol buffer) output. Depends on `midgard`, `baldr` and `odin`
- [Tools](https://github.com/valhalla/tools) - A set command line tools that exercise bits of functionality from the libraries above
- [Demos](https://github.com/valhalla/demos) - A set of demos which allows interacting with the service and APIs
- [Chef](https://github.com/valhalla/chef-valhalla) - This cookbook for installing and running valhalla

Building and Running Valhalla
-----------------------------

To build, install and run valhalla on Ubuntu (or other Debian based systems) try the following bash commands:

```bash
#grab all of the dependencies
sudo add-apt-repository ppa:kevinkreiser/prime-server
sudo apt-get update
sudo apt-get install autoconf automake libtool make gcc-4.9 g++-4.9 libboost1.54-all-dev protobuf-compiler libprotobuf-dev lua5.2 liblua5.2-dev git firefox libsqlite3-dev libspatialite-dev libgeos-dev libgeos++-dev libcurl4-openssl-dev libprime-server-dev

#build and install all valhalla includes, libraries and binaries
for repo in midgard baldr sif meili skadi mjolnir loki odin thor tyr tools; do
  git clone --recurse-submodules https://github.com/valhalla/$repo.git
  pushd $repo
  ./autogen.sh
  ./configure CPPFLAGS="-DBOOST_SPIRIT_THREADSAFE -DBOOST_NO_CXX11_SCOPED_ENUMS"
  make -j$(nproc)
  sudo make install
  popd
done
git clone --recurse-submodules https://github.com/valhalla/conf.git

#download some data and make tiles out of it
#note: you can feed multiple extracts into pbfgraphbuilder
wget http://download.geofabrik.de/europe/switzerland-latest.osm.pbf http://download.geofabrik.de/europe/liechtenstein-latest.osm.pbf
sudo mkdir -p /data/valhalla
sudo chown `whoami` /data/valhalla
rm -rf /data/valhalla/*
#TODO: run valhalla_build_addmins?
LD_LIBRARY_PATH=/usr/lib:/usr/local/lib valhalla_build_tiles -c conf/valhalla.json switzerland-latest.osm.pbf liechtenstein-latest.osm.pbf

#grab the demos repo and open up the point and click routing sample
git clone --depth=1 --recurse-submodules --single-branch --branch=gh-pages https://github.com/valhalla/demos.git
firefox demos/routing/index.html &
#NOTE: set the environment pulldown to 'localhost' to point it at your own server

#start up the server
LD_LIBRARY_PATH=/usr/lib:/usr/local/lib valhalla_route_service conf/valhalla.json

#HAVE FUN!
```

Command Line Tools
------------------
####valhalla_run_route
A C++ application that will create a route path with guidance instructions for the specified route request.
```
#Usage:
./valhalla_run_route -j '<JSON_ROUTE_REQUEST>' --config <CONFIG_FILE>
#Example:
./valhalla_run_route -j '{"locations":[{"lat":40.285488,"lon":-76.650597,"type":"break","city":"Hershey","state":"PA"},{"lat":40.794025,"lon":-77.860695,"type":"break","city":"State College","state":"PA"}],"costing":"auto","directions_options":{"units":"miles"}}' --config ../conf/valhalla.json
```

####valhalla_route_service
A C++ service that can be used to test Valhalla locally.
```
#Usage:
./valhalla_route_service <CONFIG_FILE>
#Example:
./valhalla_route_service conf/valhalla.json
#Localhost URL
http://localhost:8002/route?json={"locations":[{"lat":40.285488,"lon":-76.650597,"type":"break","city":"Hershey","state":"PA"},{"lat":40.794025,"lon":-77.860695,"type":"break","city":"State College","state":"PA"}],"costing":"auto","directions_options":{"units":"miles"}}
```

Batch Script Tool
-----------------
- [Batch Run_Route](https://github.com/valhalla/tools/blob/master/run_route_scripts/README.md)

