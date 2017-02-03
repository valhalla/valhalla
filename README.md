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


Valhalla is an open source routing engine and accompanying libraries for use with OpenStreetMap data. Valhalla also includes tools like time+distance matrix computation, isochrones, elevation sampling, map matching and tour optimization (Traveling Salesman).

Build Status
------------

[![Circle CI](https://circleci.com/gh/valhalla/valhalla.svg?style=svg)](https://circleci.com/gh/valhalla/valhalla)

Overview
--------

The are several key features that we hope can differentiate the Valhalla project from other routing and network analysis engines. They are:

- Open source software, on open source data with a very liberal license. Should allow for transparency in development, encourage contribution and community input, and foster use in other projects.
- Tiled hierarchical data structure. Should allow users to have a small memory footprint on memory constrained devices, enable offline routing, provide a means for regional extracts and partial updates.
- Dynamic, runtime costing of edges and vertices within the graph via a plugin architecture. Should allow for customization and alternate route generation.
- C++ based API. Should allow for cross compilation of the various pieces to enable routing on offline portable devices.
- A plugin based narrative and maneuver generation architecture. Should allow for generation that is customized either to the administrative area or to the target locale.
- Multi-modal and time-based routes. Should allow for mixing auto, pedestrian, bike and public transportation in the same route or setting a time by which one must arrive at a location.

Organization
--------

The Valhalla organization is comprised of several repositories each responsible for a different function. The layout of the various repositories is as follows:

- [Midgard](https://github.com/valhalla/midgard) - Basic geographic and geometric algorithms for use in the various other projects.
- [Baldr](https://github.com/valhalla/baldr) - The base data structures for accessing and caching tiled route data.
- [Sif](https://github.com/valhalla/sif) - Library used in costing of graph nodes and edges. This can be used as input to `loki` and `thor`.
- [Skadi](https://github.com/valhalla/skadi) - Library and service for accessing elevation data. This can be used as input to `mjolnir` or as a standalone service.
- [Mjolnir](https://github.com/valhalla/mjolnir) - Tools for turning open data into Valhalla graph tiles.
- [Loki](https://github.com/valhalla/loki) - Library used to search graph tiles and correlate input locations to an entity within a tile. This correlated entity (edge or vertex) can be used as input to `thor`.
- [Meili](https://github.com/valhalla/meili) - Library used to for map-matching.
- [Thor](https://github.com/valhalla/thor) - Library used to generate a path through the graph tile hierarchy.  This path and attribution along the path can be used as input to `odin`.
- [Odin](https://github.com/valhalla/odin) - Library used to generate maneuvers and narrative based on a path. This set of directions information can be used as input to `tyr`.
- [Tyr](https://github.com/valhalla/tyr) - Service used to handle http requests for a route communicating with all of the other valhalla APIs. The service will format output from `odin` and support json (and eventually protocol buffer) output.
- [conf](https://github.com/valhalla/conf) - Runtime configuration files.
- [Tools](https://github.com/valhalla/tools) - A set command line tools that exercise bits of functionality from the libraries above and provide the basis for quality testing and performance benchmarking.
- [Demos](https://github.com/valhalla/demos) - A set of demos which allows interacting with the service and APIs.
- [Chef](https://github.com/valhalla/chef-valhalla) - A chef cookbook demonstrating how to deploy the valhalla stack to a virtual machine (sample vagrant file included).

Documentation
--------

Technical documentation for the various components of the library can be found here: [docs](docs). Service API documentation as well as links to a variety of technical descriptions are provided within the [valhalla-docs](https://github.com/valhalla/valhalla-docs) repository.

Building and Using
------------------

Valhalla uses the [GNU Build System](http://www.gnu.org/software/automake/manual/html_node/GNU-Build-System.html) to configure and build itself. To install on a Debian or Ubuntu system you need to install its dependencies with:

    if [[ $(grep -cF trusty /etc/lsb-release) > 0 ]]; then
      sudo add-apt-repository -y ppa:kevinkreiser/libsodium
      sudo add-apt-repository -y ppa:kevinkreiser/libpgm
      sudo add-apt-repository -y ppa:kevinkreiser/zeromq3
      sudo add-apt-repository -y ppa:kevinkreiser/czmq
    fi
    sudo add-apt-repository -y ppa:kevinkreiser/prime-server
    sudo apt-get update
    sudo apt-get install -y autoconf automake make libtool pkg-config g++ gcc jq lcov protobuf-compiler vim-common libboost-all-dev libboost-all-dev libcurl4-openssl-dev libprime-server0.6.3-dev libprotobuf-dev prime-server0.6.3-bin
    #if you plan to compile with data building support, see below for more info
    sudo apt-get install -y libgeos-dev libgeos++-dev liblua5.2-dev libspatialite-dev libsqlite3-dev lua5.2
    #if you plan to compile with python bindings, see below for more info
    python-all-dev

And then run to install it:

    ./autogen.sh
    ./configure
    make test -j$(nproc)
    sudo make install

Please see `./configure --help` for more options on how to control the build process. There are a few notable options that you might want to try out:

* `--enable-data-tools=no` will disable building any of the components (library bits, executables and tests) which can be used to create the data that the services run on. This can be useful in embedded situations where you really dont need some of the dependencies above.
* `--enable-static=yes` will enable building of static libvalhalla.la which could be useful for embedded applications
* `--enable-python-bindings=no` will build python bindings for valhalla. Embedded applications would probably rather turn this off.

The build will produce libraries, headers and binaries which you are free to use for your own projects. To simplify the inclusion of the libvalhalla in another autotoolized project you may make use of `pkg-config` within your own `configure.ac` to check for the existance of a recent version of the library. Something like this should suffice:

    PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla >= 2.0.6])

Valhalla, and all of the projects under the Valhalla organization use the [MIT License](COPYING).

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
