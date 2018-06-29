## :rotating_light: UNDER CONSTRUCTION - MASTER NOT STABLE :rotating_light:
While Valhalla on v3.0 is under development, we recommend that you use the last stable release, [2.6.1](https://github.com/valhalla/valhalla/releases/tag/2.6.1) until further notice.

Pull requests against master are still welcome, though one may need to pay extra attention to the breaking changes in progress.

---



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


Valhalla is an open source routing engine and accompanying libraries for use with OpenStreetMap data. Valhalla also includes tools like time+distance matrix computation, isochrones, elevation sampling, map matching and tour optimization (Travelling Salesman).

Build Status
------------

| Linux | Code Coverage |
| ----- | ------------- |
| [![Circle CI](https://circleci.com/gh/valhalla/valhalla.svg?style=svg)](https://circleci.com/gh/valhalla/valhalla) | [![codecov](https://codecov.io/gh/valhalla/valhalla/branch/master/graph/badge.svg)](https://codecov.io/gh/valhalla/valhalla) |



License
-------

Valhalla, and all of the projects under the Valhalla organization, use the [MIT License](COPYING).

Overview
--------

There are several key features that we hope can differentiate the Valhalla project from other routing and network analysis engines. They are:

- Open source software, on open source data with a very liberal license. Should allow for transparency in development, encourage contribution and community input, and foster use in other projects.
- Tiled hierarchical data structure. Should allow users to have a small memory footprint on memory constrained devices, enable offline routing, provide a means for regional extracts and partial updates.
- Dynamic, runtime costing of edges and vertices within the graph via a plugin architecture. Should allow for customization and alternate route generation.
- C++ based API. Should allow for cross compilation of the various pieces to enable routing on offline portable devices.
- A plugin based narrative and manoeuvre generation architecture. Should allow for generation that is customized either to the administrative area or to the target locale.
- Multi-modal and time-based routes. Should allow for mixing auto, pedestrian, bike and public transportation in the same route or setting a time by which one must arrive at a location.

Organization
--------

The Valhalla organization is comprised of several library modules each responsible for a different function. The layout of the various modules is as follows:

- [Midgard](https://github.com/valhalla/valhalla/tree/master/valhalla/midgard) - Basic geographic and geometric algorithms for use in the various other projects.
- [Baldr](https://github.com/valhalla/valhalla/tree/master/valhalla/baldr) - The base data structures for accessing and caching tiled route data.
- [Sif](https://github.com/valhalla/valhalla/tree/master/valhalla/sif) - Library used in costing of graph nodes and edges. This can be used as input to `loki` and `thor`.
- [Skadi](https://github.com/valhalla/valhalla/tree/master/valhalla/skadi) - Library and service for accessing elevation data. This can be used as input to `mjolnir` or as a standalone service.
- [Mjolnir](https://github.com/valhalla/valhalla/tree/master/valhalla/mjolnir) - Tools for turning open data into Valhalla graph tiles.
- [Loki](https://github.com/valhalla/valhalla/tree/master/valhalla/loki) - Library used to search graph tiles and correlate input locations to an entity within a tile. This correlated entity (edge or vertex) can be used as input to `thor`.
- [Meili](https://github.com/valhalla/valhalla/tree/master/valhalla/meili) - Library used to for map-matching.
- [Thor](https://github.com/valhalla/valhalla/tree/master/valhalla/thor) - Library used to generate a path through the graph tile hierarchy.  This path and attribution along the path can be used as input to `odin`.
- [Odin](https://github.com/valhalla/valhalla/tree/master/valhalla/odin) - Library used to generate manoeuvres and narrative based on a path. This set of directions information can be used as input to `tyr`.
- [Tyr](https://github.com/valhalla/valhalla/tree/master/valhalla/tyr) - Service used to handle http requests for a route communicating with all of the other valhalla APIs. The service will format output from `odin` and support json (and eventually protocol buffer) output.
- [Tools](https://github.com/valhalla/valhalla/tree/master/src) - A set command line tools that exercise bits of functionality from the library components above and provide the basis for quality testing and performance benchmarking.
- [Demos](https://github.com/valhalla/demos) - A set of demos which allows interacting with the service and APIs.

Documentation
--------

Technical documentation for the various components of the library can be found here: [docs](docs). Service API documentation as well as links to a variety of technical descriptions are provided within the [valhalla-docs](https://github.com/valhalla/valhalla-docs) repository.

Get Valhalla from Personal Package Archive (PPA)
------------------------------------------------

If you are running Ubuntu (trusty or xenial) Valhalla can be installed quickly and easily via PPA. Try the following:

```bash
# grab all of the valhalla software from ppa
sudo add-apt-repository -y ppa:valhalla-core/valhalla
sudo apt-get update
sudo apt-get install -y valhalla-bin
```

Building from Source
--------------------

Valhalla uses CMake as build system.

To install on a Debian or Ubuntu system you need to install its dependencies with:

```bash
sudo add-apt-repository -y ppa:valhalla-core/valhalla
sudo apt-get update
sudo apt-get install -y cmake make libtool pkg-config g++ gcc jq lcov protobuf-compiler vim-common libboost-all-dev libboost-all-dev libcurl4-openssl-dev zlib1g-dev liblz4-dev libprime-server0.6.3-dev libprotobuf-dev prime-server0.6.3-bin
#if you plan to compile with data building support, see below for more info
sudo apt-get install -y libgeos-dev libgeos++-dev liblua5.2-dev libspatialite-dev libsqlite3-dev lua5.2 wget
if [[ $(grep -cF xenial /etc/lsb-release) > 0 ]]; then sudo apt-get install -y libsqlite3-mod-spatialite; fi
#if you plan to compile with python bindings, see below for more info
sudo apt-get install -y python-all-dev
```

To install on macOS, you need to install its dependencies with [Homebrew](http://brew.sh):

```bash
# install dependencies (czmq is required by prime_server)
brew install cmake libtool protobuf-c boost-python libspatialite pkg-config sqlite3 lua jq curl wget czmq lz4
```

Then clone and build [`prime_server`](https://github.com/kevinkreiser/prime_server#build-and-install).

After getting the dependencies install it with:

```bash
git submodule update --init --recursive
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
// to install with node bindings
make copy_node_artifacts
```

Important build options include:

| Option | Behavior |
|--------|----------|
| `-DENABLE_DATA_TOOLS` (`On`/`Off`) | Build the data preprocessing tools|
| `-DENABLE_PYTHON_BINDINGS` (`On`/`Off`) | Build the python bindings|
| `-DENABLE_SERVICES` (`On` / `Off`) | Build the HTTP service|
| `-DBUILD_SHARED_LIBS` (`On` / `Off`) | Build static or shared libraries|
| `-DENABLE_NODE_BINDINGS` (`ON` / `OFF`) | Build the node bindings (defaults to on)|

For more build options run the interactive GUI:

```bash
cd build
cmake ..
ccmake ..
```

For more information on binaries, see [Command Line Tools](#command-line-tools) section below and the [docs](docs).

Running
-------

The following bash should be enough to make some routing data and start a server using it. (Note - if you would like to run an elevation lookup service with Valhalla follow the instructions [here](docs/elevation.md)).

```bash
#download some data and make tiles out of it
#NOTE: you can feed multiple extracts into pbfgraphbuilder
wget http://download.geofabrik.de/europe/switzerland-latest.osm.pbf http://download.geofabrik.de/europe/liechtenstein-latest.osm.pbf
#get the config and setup
mkdir -p valhalla_tiles
valhalla_build_config --mjolnir-tile-dir ${PWD}/valhalla_tiles --mjolnir-tile-extract ${PWD}/valhalla_tiles.tar --mjolnir-timezone ${PWD}/valhalla_tiles/timezones.sqlite --mjolnir-admin ${PWD}/valhalla_tiles/admins.sqlite > valhalla.json
#build routing tiles
#TODO: run valhalla_build_admins?
valhalla_build_tiles -c valhalla.json switzerland-latest.osm.pbf liechtenstein-latest.osm.pbf
#tar it up for running the server
find valhalla_tiles | sort -n | tar cf valhalla_tiles.tar --no-recursion -T -

#grab the demos repo and open up the point and click routing sample
git clone --depth=1 --recurse-submodules --single-branch --branch=gh-pages https://github.com/valhalla/demos.git
firefox demos/routing/index-internal.html &
#NOTE: set the environment pulldown to 'localhost' to point it at your own server

#start up the server
valhalla_service valhalla.json 1
#curl it directly if you like:
curl http://localhost:8002/route --data '{"locations":[{"lat":40.285488,"lon":-76.650597,"type":"break","city":"Hershey","state":"PA"},{"lat":40.794025,"lon":-77.860695,"type":"break","city":"State College","state":"PA"}],"costing":"auto","directions_options":{"units":"miles"}}' | jq '.'

#HAVE FUN!
```

Contributing
------------

We welcome contributions to valhalla. If you would like to report an issue, or even better fix an existing one, please use the [valhalla issue tracker](https://github.com/valhalla/valhalla/issues) on GitHub.

If you would like to make an improvement to the code, please be aware that all valhalla projects are written mostly in C++11.  We use `clang-format` v7.0 to format the code. We welcome contributions as pull requests to the [repository](https://github.com/valhalla/valhalla) and highly recommend that your pull request include a test to validate the addition/change of functionality.

Note that our CI system checks that code formatting is consistent, and the build will fail if formatting rules aren't followed.  Please run `./scripts/format.sh` over your code before committing, to auto-format it in the projects preferred style.

Also note that we run some `clang-tidy` linting over the code as well (see `.clang-tidy` for the list of rules enforced).  You can run `./scripts/tidy.sh` over the code before committing to ensure you haven't added any of the common problems we check for (Note: `./scripts/tidy.sh` requires the exitence of a `compile_commands.json` database.  You can generate this file by running `bear make` instead of just `make`.  The `bear` tool is installable on Ubuntu-based systems with `apt-get install bear`, and on macOS with `brew install bear`).

Tests
-----

We highly encourage running and updating the tests to make sure no regressions have been made. We use the Automake test suite to run our tests by simply making the `check` target:

    make check

To run an individual test, `make run-<test name>` from the build directory or `./test/<testname>`

Coverage reports are automatically generated using codecov for each pull request, but you can also build them locally by passing `-DENABLE_COVERAGE=On` and running `make coverage`.

Command Line Tools
------------------
#### valhalla_run_route
A C++ application that will create a route path with guidance instructions for the specified route request.

```bash
#Usage:
./valhalla_run_route -j '<JSON_ROUTE_REQUEST>' --config <CONFIG_FILE>
#Example:
./valhalla_run_route -j '{"locations":[{"lat":40.285488,"lon":-76.650597,"type":"break","city":"Hershey","state":"PA"},{"lat":40.794025,"lon":-77.860695,"type":"break","city":"State College","state":"PA"}],"costing":"auto","directions_options":{"units":"miles"}}' --config ../conf/valhalla.json
```

Batch Script Tool
-----------------
- [Batch Run_Route](./run_route_scripts/README.md)
