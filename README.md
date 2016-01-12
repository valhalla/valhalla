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

Overview
--------

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

Building and Running
--------------------

To build, install and run valhalla on Ubuntu (or other Debian based systems) try the following bash commands:

    #add ppa for newer compiler
    sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
    sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/ubuntu-toolchain-r-test-$(lsb_release -c -s).list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
    
    #grab all of the dependencies
    sudo apt-get install autoconf automake libtool make gcc-4.9 g++-4.9 libboost1.54-dev libboost-program-options1.54-dev libboost-filesystem1.54-dev libboost-system1.54-dev libboost-thread1.54-dev  protobuf-compiler libprotobuf-dev lua5.2 liblua5.2-dev git firefox libsqlite3-dev libspatialite-dev libgeos-dev libgeos++-dev libcurl4-openssl-dev
    
    #use newer compiler
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 90
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 90
    
    #grab the latest zmq library:
    rm -rf libzmq
    git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/zeromq/libzmq.git
    pushd libzmq
    ./autogen.sh
    ./configure --without-libsodium --without-documentation
    make -j
    sudo make install
    popd
    
    #grab prime_server API:
    rm -rf prime_server
    git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/kevinkreiser/prime_server.git
    pushd prime_server
    ./autogen.sh
    ./configure
    make -j
    sudo make install
    popd

    #build and install all valhalla includes, libraries and binaries
    for repo in midgard baldr sif skadi mjolnir loki odin thor tyr tools; do
      git clone --recurse-submodules https://github.com/valhalla/$repo.git
      pushd $repo
      ./autogen.sh
      ./configure CPPFLAGS=-DBOOST_SPIRIT_THREADSAFE
      make -j
      sudo make install
      popd
    done

    #download some data and make tiles out of it
    #note: you can feed multiple extracts into pbfgraphbuilder
    pushd mjolnir
    wget http://download.geofabrik.de/europe/switzerland-latest.osm.pbf http://download.geofabrik.de/europe/liechtenstein-latest.osm.pbf
    sudo mkdir -p /data/valhalla
    sudo chown `whoami` /data/valhalla
    rm -rf /data/valhalla/*
    #TODO: run pbfadminbuilder?
    LD_LIBRARY_PATH=/usr/lib:/usr/local/lib pbfgraphbuilder -c conf/valhalla.json switzerland-latest.osm.pbf liechtenstein-latest.osm.pbf
    popd

    #grab the demos repo and open up the point and click routing sample
    git clone --depth=1 --recurse-submodules --single-branch --branch=gh-pages https://github.com/valhalla/demos.git
    firefox demos/routing/index.html &
    #NOTE: set the environment pulldown to 'localhost' to point it at your own server

    #start up the server
    LD_LIBRARY_PATH=/usr/lib:/usr/local/lib tools/tyr_simple_service tools/conf/valhalla.json

    #HAVE FUN!

Using
-----

The build will install libraries, headers, executables and python modules for use in running the service and cutting tiles, however you are free to use any of these for your own projects as well. To simplify the inclusion of the these libraries in another autotoolized project you may make use of the various [valhalla_* m4s](m4/) in your own `configure.ac` file. For an exmample of this please have a look at `configure.ac` in another one of the valhalla projects. This cookbook, and all of the projects under the Valhalla organization use the [MIT License](LICENSE.).

Contributing
------------

We welcome contributions to the cookbook. If you would like to report an issue, or even better fix an existing one, please use the [chef issue tracker](https://github.com/valhalla/chef-valhalla/issues) on GitHub.

If you would like to make an improvement to the cookbook, please be aware that we adhere to strict style guidlines and enforce them using `rake`. If you would like to make an improvement to the C++ APIs, please be aware that all valhalla projects are written mostly in C++11, in the K&R (1TBS variant) with two spaces as indentation. We generally follow this [c++ style guide](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html). We welcome contributions as pull requests to the repository linked above and highly recommend that your pull request include a test to validate the addition/change of functionality.

As noted above we forsee several good places to start in terms of contributing. We've outlined plugin architectures for the path finding algorithm's costing model, maneuver generation and narrative generation. As these things take shape please look to the [Thor](https//github.com/valhalla/thor) and [Odin](https//github.com/valhalla/odin) repositories' documentation. These should provide more information on how to write your own plugins for custom functionality.

Tests
---------

We highly encourage running and updating the tests to make sure no regressions have been made. We use the Automake test suite to run our tests by simply making the `check` target:

    make check

You can also build a test coverage report. This requires that the packages `lcov`, `gcov` and `genhtml` be installed. On Ubuntu you can get these with:

    sudo apt-get install lcov

To make the coverage report, configure the build for it:

    ./configure --enable-coverage

And generate an HTML coverage report in the `coverage/` directory:

    make coverage-report

Note also that, because calculating the coverage requires compiler support, you will need to clean any object files from a non-coverage build by running `make clean` before `make coverage-report`.
