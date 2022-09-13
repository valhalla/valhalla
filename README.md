


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

## Build Status

| Linux/MacOs | Windows | MinGW64 | Code Coverage |
| ----------- | ------- | ------------- | ------------- |
| [![Circle CI](https://circleci.com/gh/valhalla/valhalla/tree/master.svg?style=svg)](https://circleci.com/gh/valhalla/valhalla/tree/master) | [![Build Status](https://dev.azure.com/valhalla1/valhalla/_apis/build/status/valhalla.valhalla?branchName=master)](https://dev.azure.com/valhalla1/valhalla/_build/latest?definitionId=1&branchName=master) | ![Valhalla MinGW Build](https://github.com/valhalla/valhalla/workflows/Valhalla%20MinGW%20Build/badge.svg) | [![codecov](https://codecov.io/gh/valhalla/valhalla/branch/master/graph/badge.svg)](https://codecov.io/gh/valhalla/valhalla) |


## License

Valhalla, and all of the projects under the Valhalla organization, use the [MIT License](COPYING).  Avatar/logo by [Jordan](https://www.instagram.com/jaykaydraws/)

## Overview

There are several key features that we hope can differentiate the Valhalla project from other routing and network analysis engines. They are:

- Open source software, on open source data with a very liberal license. Should allow for transparency in development, encourage contribution and community input, and foster use in other projects.
- Tiled hierarchical data structure. Should allow users to have a small memory footprint on memory constrained devices, enable offline routing, provide a means for regional extracts and partial updates.
- Dynamic, runtime costing of edges and vertices within the graph via a plugin architecture. Should allow for customization and alternate route generation.
- C++ based API. Should allow for cross compilation of the various pieces to enable routing on offline portable devices.
- A plugin based narrative and manoeuvre generation architecture. Should allow for generation that is customized either to the administrative area or to the target locale.
- Multi-modal and time-based routes. Should allow for mixing auto, pedestrian, bike and public transportation in the same route or setting a time by which one must arrive at a location.

## Demo Server

[FOSSGIS e.V.](https://fossgis.de) hosts a demo server which is open to the public and includes a full planet graph on https://valhalla.openstreetmap.de with an [open-source web app](https://github.com/gis-ops/valhalla-app). The HTTP API is accessible on a slightly different subdomain, e.g. https://valhalla1.openstreetmap.de/isochrone. Usage of the demo server follows the usual fair-usage policy as OSRM & Nominatim demo servers (somewhat enforced by [rate limits](https://github.com/valhalla/valhalla/discussions/3373#discussioncomment-1644713)).

## Platform Compatibility

Valhalla is fully functional on many Linux and Mac OS distributions.

In Windows all functionality is not yet fully supported. Building the Valhalla library works flawlessly, as well as the following application modules:

- `TOOLS`: utilities to query and benchmark various components
- `DATA_TOOLS`: utilities to build input data and handle transit
- `PYTHON_BINDINGS`: use all actions (route, isochrones, matrix etc) via the Valhalla Python library (needs a full (i.e. development) Python distribution in the `PATH`)

## Organization

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

## Documentation

Documentation is stored in the `docs/` folder in this GitHub repository. It can be viewed at [valhalla.readthedocs.io](https://valhalla.readthedocs.io/).

## Installation

### Docker

Checkout our `run-*` docker containers here: https://hub.docker.com/r/valhalla/valhalla/tags

### Build Configuration

Valhalla uses CMake as build system. When compiling with gcc (GNU Compiler Collection), version 5 or newer is supported.

Important build options include:

| Option | Behavior |
|--------|----------|
| `-DENABLE_DATA_TOOLS` (`On`/`Off`) | Build the data preprocessing tools|
| `-DENABLE_PYTHON_BINDINGS` (`On`/`Off`) | Build the python bindings|
| `-DENABLE_SERVICES` (`On` / `Off`) | Build the HTTP service|
| `-DBUILD_SHARED_LIBS` (`On` / `Off`) | Build static or shared libraries|
| `-DENABLE_COMPILER_WARNINGS` (`ON` / `OFF`) | Build with common compiler warnings (defaults to off)|
| `-DENABLE_WERROR` (`ON` / `OFF`) | Treat compiler warnings as errors  (defaults to off). Requires `-DENABLE_COMPILER_WARNINGS=ON` to take effect.|
| `-DENABLE_BENCHMARKS` (`ON` / `OFF`) | Enable microbenchmarking  (defaults to on).|
| `-DENABLE_SANITIZERS` (`ON` / `OFF`) | Build with all the integrated sanitizers (defaults to off).|
| `-DENABLE_ADDRESS_SANITIZER` (`ON` / `OFF`) | Build with address sanitizer (defaults to off).|
| `-DENABLE_UNDEFINED_SANITIZER` (`ON` / `OFF`) | Build with undefined behavior sanitizer (defaults to off).|

For more build options run the interactive GUI or have a look at the root's [`CmakeLists.txt`](./CMakeLists.txt):

```bash
cd build
cmake ..
ccmake ..
```

If you're building on Apple Silicon and use the Rosetta terminal (see below), you might need to additionally specify the appropriate options:

```
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES="x86_64"
```

### Building from Source - Linux

To install on a Debian or Ubuntu system you need to install its dependencies with:

```bash
sudo add-apt-repository -y ppa:valhalla-core/valhalla
sudo apt-get update
sudo apt-get install -y cmake make libtool pkg-config g++ gcc curl unzip jq lcov protobuf-compiler vim-common locales libcurl4-openssl-dev zlib1g-dev liblz4-dev libprime-server-dev libprotobuf-dev prime-server-bin
#if you plan to compile with data building support, see below for more info
sudo apt-get install -y libgeos-dev libgeos++-dev libluajit-5.1-dev libspatialite-dev libsqlite3-dev wget sqlite3 spatialite-bin python3-shapely
source /etc/lsb-release
if [[ $(python3 -c "print(int($DISTRIB_RELEASE > 15))") > 0 ]]; then sudo apt-get install -y libsqlite3-mod-spatialite; fi
#if you plan to compile with python bindings, see below for more info
sudo apt-get install -y python-all-dev
```

Now you can build and install Valhalla, e.g. 

```bash
# will build to ./build
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j$(nproc)
sudo make -C build install
```

### Building from Source - macOS

#### Configuring Rosetta for ARM64 MacBook

Check your architecture typing `arch` in the terminal. In case the result is `arm64` set up Rosetta terminal to emulate x86_64 behavior. Otherwise, skip this step.

1. Go to `Finder > Application > Utilities`.
2. Select `Terminal` and right-click on it, then choose `Duplicate`.
3. Rename the duplicated app `Rosetta Terminal`.
4. Now select `Rosetta Terminal` application, right-click and choose `Get Info` .
5. Check the box for `Open using Rosetta`, then close the `Get Info` window.
6. Make shure you get `i386` after typing `arch` command in  `Rosetta Terminal`.
7. Now it fully supports Homebrew and other x86_64 command line applications.

Install [Homebrew](http://brew.sh) in the `Rosetta Terminal` app and update the aliases.

```
echo "alias ibrew='arch -x86_64 /usr/local/bin/brew'" >> ~/.zshrc
echo "alias mbrew='arch -arm64e /opt/homebrew/bin/brew'" >> ~/.zshrc
```

You will use them to specify the platform when installing a library. Note: use `ibrew` in `Rosetta Terminal` to install all dependencies for `valhalla` and `prime_server` projects.

**_NOTE:_** If when installing packages below you get message `attempting to link with file built for macOS-arm64`, you can remove already installed packages for arm64 i.e. `mbrew uninstall ...`. Also, if there are problems with individual packages, you can install them from sources e.g. [geos](https://github.com/libgeos/geos) or [sqlite](https://www.sqlite.org/download.html).

**_NOTE:_** It is possible to build Valhalla natively for Apple Silicon, but some dependencies(e.g. LuaJIT) don't have stable versions supporting Apple Silicon and have to be built and installed manually from source.

#### Installing dependencies

To install valhalla on macOS, you need to install its dependencies with [Homebrew](http://brew.sh):

```bash
# install dependencies (automake & czmq are required by prime_server)
brew install automake cmake libtool protobuf-c libspatialite pkg-config sqlite3 jq curl wget czmq lz4 spatialite-tools unzip luajit
# following packages are needed for running Linux compatible scripts
brew install bash coreutils binutils
# Update your PATH env variable to include /usr/local/opt/binutils/bin:/usr/local/opt/coreutils/libexec/gnubin
```

Now, clone the Valhalla repository

```bash
git clone --recurse-submodules https://github.com/valhalla/valhalla.git
```

Then, build [`prime_server`](https://github.com/kevinkreiser/prime_server#build-and-install).

After getting the dependencies install it with e.g.:

```bash
# will build to ./build
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j$(sysctl -n hw.physicalcpu)
sudo make -C build install
```

### Building from Source - Windows

It's recommended to work with the following toolset:
- Visual Studio with C++ support
- Visual Studio Code (easier and lighter to handle)
- [vcpkg](https://github.com/Microsoft/vcpkg) to install packages
- [CMake](https://cmake.org/download/)

1. Install the following packages with `vcpkg` and your platform triplet (e.g. `x64-windows`). Note, you can remove all packages after `zlib` in `.\.vcpkg_deps.txt` if you don't want to build `TOOLS` & `DATA_TOOLS`:
```
# Basic packages
git -C C:\path\to\vcpkg checkout f4bd6423
cd C:\path\to\project
C:\path\to\vcpkg.exe --triplet x64-windows install "@.vcpkg_deps.txt"
```
2. Let CMake configure the build with the required modules enabled. **Note**, you have to manually link LuaJIT for some reason, e.g. the final command for `x64` could look like
```
"C:\Program Files\CMake\bin\cmake.EXE" --no-warn-unused-cli -DENABLE_TOOLS=ON -DENABLE_DATA_TOOLS=ON -DENABLE_PYTHON_BINDINGS=ON -DENABLE_HTTP=ON -DENABLE_CCACHE=OFF -DENABLE_SERVICES=OFF -DENABLE_BENCHMARKS=OFF -DENABLE_TESTS=OFF -DLUA_LIBRARIES=path\to\vcpkg\installed\x64-windows\lib\lua51.lib -DLUA_INCLUDE_DIR=path\to\vcpkg\installed\x64-windows\include\luajit -DVCPKG_TARGET_TRIPLET=x64-windows -DCMAKE_TOOLCHAIN_FILE=path\to\vcpkg\scripts\buildsystems\vcpkg.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -Hpath/to/project -Bpath/to/project/build -G "Visual Studio 16 2019" -T host=x64 -A x64
```
3. Run the build for all targets.
```
cd C:\path\to\project
cmake -B build .
```

## Running

The following bash should be enough to make some routing data and start a server using it. (Note - if you would like to run an elevation lookup service with Valhalla follow the instructions [here](docs/elevation.md)).

```bash
#download some data and make tiles out of it
#NOTE: you can feed multiple extracts into pbfgraphbuilder
wget http://download.geofabrik.de/europe/switzerland-latest.osm.pbf http://download.geofabrik.de/europe/liechtenstein-latest.osm.pbf
#get the config and setup
mkdir -p valhalla_tiles
valhalla_build_config --mjolnir-tile-dir ${PWD}/valhalla_tiles --mjolnir-tile-extract ${PWD}/valhalla_tiles.tar --mjolnir-timezone ${PWD}/valhalla_tiles/timezones.sqlite --mjolnir-admin ${PWD}/valhalla_tiles/admins.sqlite > valhalla.json
#build timezones.sqlite to support time-dependent routing
valhalla_build_timezones > valhalla_tiles/timezones.sqlite
#build routing tiles
#TODO: run valhalla_build_admins?
valhalla_build_tiles -c valhalla.json switzerland-latest.osm.pbf liechtenstein-latest.osm.pbf
#tar it up for running the server
#either run this to build a tile index for faster graph loading times
valhalla_build_extract -c valhalla.json -v
#or simply tar up the tiles
find valhalla_tiles | sort -n | tar cf valhalla_tiles.tar --no-recursion -T -

#grab the demos repo and open up the point and click routing sample
git clone --depth=1 --recurse-submodules --single-branch --branch=gh-pages https://github.com/valhalla/demos.git
firefox demos/routing/index-internal.html &
#NOTE: set the environment pulldown to 'localhost' to point it at your own server

#start up the server
valhalla_service valhalla.json 1
#curl it directly if you like:
curl http://localhost:8002/route --data '{"locations":[{"lat":47.365109,"lon":8.546824,"type":"break","city":"Zürich","state":"Altstadt"},{"lat":47.108878,"lon":8.394801,"type":"break","city":"6037 Root","state":"Untere Waldstrasse"}],"costing":"auto","directions_options":{"units":"miles"}}' | jq '.'

#HAVE FUN!
```

For more information on binaries, see [Command Line Tools](#command-line-tools) section below and the [docs](docs).

## Contributing

We welcome contributions to valhalla. If you would like to report an issue, or even better fix an existing one, please use the [valhalla issue tracker](https://github.com/valhalla/valhalla/issues) on GitHub. We organize one hour each week to discuss open pull requests where everyone is welcome to join, see [our wiki](https://github.com/valhalla/valhalla/wiki/Open-Review-Days).

If you would like to make an improvement to the code, please be aware that all valhalla projects are written mostly in C++11.  We use `clang-format` v7.0 to format the code. We welcome contributions as pull requests to the [repository](https://github.com/valhalla/valhalla) and highly recommend that your pull request include a test to validate the addition/change of functionality.

Note that our CI system checks that code formatting is consistent, and the build will fail if formatting rules aren't followed.  Please run `./scripts/format.sh` over your code before committing, to auto-format it in the projects preferred style. To spare yourself (and the CIs) pure `format` commits, you can register it as a pre-commit hook so it lints your changes in-place (and will fail if files were changed, so you'll need to stage and commit again):

```
cat ./scripts/format.sh > .git/hooks/pre-commit && tail -n +7 scripts/error_on_dirty.sh >> .git/hooks/pre-commit
chmod +x .git/hooks/pre-commit
```

Also note that we run some `clang-tidy` linting over the code as well (see `.clang-tidy` for the list of rules enforced).  You can run `./scripts/clang-tidy-only-diff.sh` over the code before committing to ensure you haven't added any of the common problems we check for (Note: `./scripts/clang-tidy-only-diff.sh` requires the exitence of a `compile_commands.json` database.  You can generate this file by running `cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=On ... && make`.

`scripts/clang-tidy-only-diff.sh` is run in CI and will fail the build if it detects any issues.

Additionally, a check with [ASan](https://clang.llvm.org/docs/AddressSanitizer.html) is run in CI. We recommend testing with ASan  and debug symbols locally prior to commiting, with the `-DENABLE_ADDRESS_SANITIZER=ON -DCMAKE_BUILD_TYPE=Debug` flags during cmake configuration. As long as leak sanitizer (which is a part of address sanitizer) is not currently supported across different platforms it is disabled in the CI. You can disable it locally with the environment variable `ASAN_OPTIONS=detect_leaks=0`.

### Tests

We highly encourage running and updating the tests to make sure no regressions have been made. We use the Automake test suite to run our tests by simply making the `check` target:

    make check

To run an individual test, `make run-<test name>` from the build directory or `./test/<testname>`

You may check some notes on [unit tests](docs/testing.md)

Coverage reports are automatically generated using codecov for each pull request, but you can also build them locally by passing `-DENABLE_COVERAGE=On` and running `make coverage`.

## Benchmarks

Valhalla includes several microbenchmarks which you can build and run using:

    make benchmarks
    make run-benchmarks

They are enabled by the `-DENABLE_BENCHMARKS=On` CMake flag and are currently only available for
Linux and MacOS.

## Command Line Tools

### `valhalla_service` aka one-shot mode

If you can't (e.g. Windows Server) or don't want to have the full-fledged HTTP API running, you can have the (almost) exact same behavior with the 'valhalla_service' executable in so-called "one-shot" mode. It's simple, just pass the config file, the action (route, isochrone, matrix etc) and the stringified JSON request (or alternatively a file containing the request to circumvent shell command length issues):

```
valhalla_service valhalla.json isochrone '{"locations":[{"lat":42.552448,"lon":1.564865}],"costing":"auto","contours":[{"time":10,"color":"ff0000"}], "show_locations":true}
# Alternatively you can pass a file with the same contents
valhalla_service valhalla.json isochrone isochrone_request.txt
```

It's important to note that all Valhalla logs for one-shot mode are piped to `stderr` while the actual JSON response will be in `stdout`. To completely silence the logs, pass `type: ""` to `midgard.logging` in the config file.


### Batch Script Tool

- [Batch Run_Route](./run_route_scripts/README.md)
