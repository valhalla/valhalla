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

[![Circle CI](https://circleci.com/gh/valhalla/valhalla.svg?style=svg)](https://circleci.com/gh/valhalla/valhalla)

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
- [Chef](https://github.com/valhalla/chef-valhalla) - A chef cookbook demonstrating how to deploy the valhalla stack to a virtual machine (sample vagrant file included).

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

Valhalla uses the [GNU Build System](http://www.gnu.org/software/automake/manual/html_node/GNU-Build-System.html) to configure and build itself.

To install on a Debian or Ubuntu system you need to install its dependencies with:

```bash
sudo add-apt-repository -y ppa:valhalla-core/valhalla
sudo apt-get update
sudo apt-get install -y autoconf automake make libtool pkg-config g++ gcc jq lcov protobuf-compiler vim-common libboost-all-dev libboost-all-dev libcurl4-openssl-dev libprime-server0.6.3-dev libprotobuf-dev prime-server0.6.3-bin
#if you plan to compile with data building support, see below for more info
sudo apt-get install -y libgeos-dev libgeos++-dev liblua5.2-dev libspatialite-dev libsqlite3-dev lua5.2
if [[ $(grep -cF xenial /etc/lsb-release) > 0 ]]; then sudo apt-get install -y libsqlite3-mod-spatialite; fi
#if you plan to compile with python bindings, see below for more info
sudo apt-get install -y python-all-dev
```

To install on macOS, you need to install its dependencies with [Homebrew](http://brew.sh):

    # install dependencies (czmq is required by prime_server)
    brew install autoconf automake libtool protobuf-c boost-python libspatialite pkg-config sqlite3 lua jq curl czmq

    # clone and build prime_server https://github.com/kevinkreiser/prime_server#build-and-install

After getting the dependencies install it with:

```bash
git submodule update --init --recursive
./autogen.sh
# on macOS you need to tell linkers how to reach home-brewed sqlite3 and curl:
# export LDFLAGS="-L/usr/local/opt/sqlite/lib/ -lsqlite3" PKG_CONFIG_PATH=/usr/local/opt/curl/lib/pkgconfig
./configure
make test -j$(nproc)
sudo make install
# Note: on macOS, a few tests involving time formatting will fail: narrativebuilder, util_odin, mapmatch
```

Please see `./configure --help` for more options on how to control the build process. There are a few notable options that you might want to try out:

* `--enable-data-tools=no` will disable building any of the components (library bits, executables and tests) which can be used to create the data that the services run on. This can be useful in embedded situations where you really don't need some of the dependencies above.
* `--enable-services=no` will disable building any of the components (library bits, executables and tests) which can be used to run valhalla as an http service. This can be useful in embedded situations where you really don't need some of the dependencies above (prime_server et al).
* `--enable-static=yes` will enable building of static libvalhalla.la which could be useful for embedded applications
* `--enable-python-bindings=no` will disable python bindings for valhalla. Embedded applications would probably rather turn this off.

The build will produce libraries, headers and binaries which you are free to use for your own projects. To simplify the inclusion of the libvalhalla in another autotoolized project you may make use of `pkg-config` within your own `configure.ac` to check for the existence of a recent version of the library. Something like this should suffice:

    PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla >= 2.0.6])

For more information on binaries, see [Command Line Tools](#command-line-tools) section below and the [docs](docs).

Running
-------

The following bash should be enough to make some routing data and start a server using it:

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
valhalla_route_service valhalla.json 1
#curl it directly if you like:
curl http://localhost:8002/route --data '{"locations":[{"lat":40.285488,"lon":-76.650597,"type":"break","city":"Hershey","state":"PA"},{"lat":40.794025,"lon":-77.860695,"type":"break","city":"State College","state":"PA"}],"costing":"auto","directions_options":{"units":"miles"}}' | jq '.'

#HAVE FUN!
```

Contributing
------------

We welcome contributions to valhalla. If you would like to report an issue, or even better fix an existing one, please use the [valhalla issue tracker](https://github.com/valhalla/valhalla/issues) on GitHub.

If you would like to make an improvement to the code, please be aware that all valhalla projects are written mostly in C++11, in the K&R (1TBS variant) with two spaces as indentation. We welcome contributions as pull requests to the [repository](https://github.com/valhalla/valhalla) and highly recommend that your pull request include a test to validate the addition/change of functionality.

Tests
-----

We highly encourage running and updating the tests to make sure no regressions have been made. We use the Automake test suite to run our tests by simply making the `check` target:

    make check

You can also build a test coverage report. This requires that the packages `lcov`, `gcov` and `genhtml` be installed. On Ubuntu you can get these with:

    sudo apt-get install lcov

To make the coverage report, configure the build for it:

    ./configure --enable-coverage

And generate an HTML coverage report in the `coverage/` directory:

    make coverage-report

Note also that, because calculating the coverage requires compiler support, you will need to clean any object files from a non-coverage build by running `make clean` before `make coverage-report`.

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
