Valhalla is an open source routing engine and accompanying libraries for use with OpenStreetMap data. Valhalla also includes tools like time+distance matrix computation, isochrones, elevation sampling, map matching and tour optimization (Travelling Salesman).

## Overview

There are several key features that we hope can differentiate the Valhalla project from other routing and network analysis engines. They are:

- Open source software.
- Tiled hierarchical data structure. Should allow users to have a small memory footprint on memory constrained devices, enable offline routing, provide a means for regional extracts and partial updates.
- Dynamic, runtime costing of edges and vertices within the graph via a plugin architecture. Should allow for customization and alternate route generation.
- C++ based API. Should allow for cross compilation of the various pieces to enable routing on offline portable devices.
- A plugin based narrative and manoeuvre generation architecture. Should allow for generation that is customized either to the administrative area or to the target locale.
- Multi-modal and time-based routes. Should allow for mixing auto, pedestrian, bike and public transportation in the same route or setting a time by which one must arrive at a location.

## Demo Server

Demo valhalla app on https://valhalla.openstreetmap.de.

## Platform Compatibility

Valhalla is fully functional on many Linux and Mac OS distributions, and is also used on iOS and Android devices.

## Organization

The Valhalla organization is comprised of several library modules each responsible for a different function:

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

Documentation is stored in the `docs/` folder in this GitHub repository. It can be viewed at [valhalla.github.io/valhalla](https://valhalla.github.io/valhalla).

## Build & Run

Valhalla uses CMake as build system. When compiling with `gcc` (GNU Compiler Collection), currently version 5 or newer is supported.

Important build options include:

| Option | Behavior |
|--------|----------|
| `-DENABLE_TOOLS` (`On`/`Off`) | Build `valhalla_service` and other utilities (defaults to on)|
| `-DENABLE_DATA_TOOLS` (`On`/`Off`) | Build the data preprocessing tools (defaults to on)|
| `-DENABLE_HTTP` (`On`/`Off`) | Build with `curl` support (defaults to on)|
| `-DENABLE_PYTHON_BINDINGS` (`On`/`Off`) | Build the python bindings (defaults to on)|
| `-DENABLE_SERVICES` (`On` / `Off`) | Build the HTTP service (defaults to on)|
| `-DENABLE_THREAD_SAFE_TILE_REF_COUNT` (`ON` / `OFF`) | If ON uses shared_ptr as tile reference (i.e. it is thread safe, defaults to off)|
| `-DENABLE_CCACHE` (`On` / `Off`) | Speed up incremental rebuilds via ccache (defaults to on)|
| `-DENABLE_BENCHMARKS` (`On` / `Off`) | Enable microbenchmarking (defaults to on)|
| `-DENABLE_TESTS` (`On` / `Off`) | Enable Valhalla tests (defaults to on)|
| `-DENABLE_COVERAGE` (`On` / `Off`) | Build with coverage instrumentalisation (defaults to off)|
| `-DBUILD_SHARED_LIBS` (`On` / `Off`) | Build static or shared libraries (defaults to off)|
| `-DENABLE_STATIC_LIBRARY_MODULES` (`On` / `Off`) | If ON builds Valhalla modules as STATIC library targets (defaults to off)|
| `-DENABLE_COMPILER_WARNINGS` (`ON` / `OFF`) | Build with common compiler warnings (defaults to off)|
| `-DENABLE_SINGLE_FILES_WERROR` (`ON` / `OFF`) | Convert compiler warnings to errors for a (growing) selection of files (defaults to on)|
| `-DENABLE_WERROR` (`ON` / `OFF`) | Treat compiler warnings as errors  (defaults to off). Requires `-DENABLE_COMPILER_WARNINGS=ON` to take effect.|
| `-DENABLE_SANITIZERS` (`ON` / `OFF`) | Build with all the integrated sanitizers (defaults to off).|
| `-DENABLE_ADDRESS_SANITIZER` (`ON` / `OFF`) | Build with address sanitizer (defaults to off).|
| `-DENABLE_UNDEFINED_SANITIZER` (`ON` / `OFF`) | Build with undefined behavior sanitizer (defaults to off).|
| `-DPREFER_SYSTEM_DEPS` (`ON` / `OFF`) | Whether to use internally vendored headers or find the equivalent external package (defaults to off).|
| `-DENABLE_GDAL` (`ON` / `OFF`) | Whether to include GDAL as a dependency (used for GeoTIFF serialization of isochrone grid) (defaults to off).|

Clone source:

	git clone --recurse-submodules https://github.com/lqh2307/valhalla.git

Chuyển sang nhánh dev:

	git checkout dev

Build image:

    docker build -t quanghuy2307/valhalla:latest .

Tạo folder mới muốn đặt data và di chuyển vào đó:

	mkdir -p \
		/home/vht/data \
		/home/vht/data/valhalla \
		/home/vht/data/valhalla/transit \
		/home/vht/data/valhalla/elevation_tiles \
		&& cd /home/vht/data

Tải OSM data source của Việt Nam:

	wget http://download.geofabrik.de/asia/vietnam-latest.osm.pbf

Tải timezone:

	wget https://github.com/evansiroky/timezone-boundary-builder/releases/download/2024a/timezones-with-oceans.shapefile.zip

Tải elevation data (thay thế {tile-name} bằng các mảnh cần tải):

	wget -P /home/vht/data/valhalla/elevation_tiles wget https://dwtkns.com/srtm30m/{tile-name}

Chạy container:

    docker run --rm -it --name valhalla -p 8002:8002 -v /home/vht/data/:/data quanghuy2307/valhalla:latest

Chạy các lệnh sau trong container:

    valhalla_build_config \
        --mjolnir-tile-dir ${PWD}/valhalla/tiles \
        --mjolnir-transit-dir ${PWD}/valhalla/transit \
        --additional-data-elevation ${PWD}/valhalla/elevation_tiles \
        --mjolnir-tile-extract ${PWD}/valhalla/tiles.tar \
        --mjolnir-traffic-extract ${PWD}/valhalla/traffic.tar \
        --mjolnir-timezone ${PWD}/valhalla/tz_world.sqlite \
        --mjolnir-admin ${PWD}/valhalla/admin.sqlite \
        --mjolnir-landmarks ${PWD}/valhalla/landmarks.sqlite \
        > valhalla/valhalla.json
    valhalla_build_timezones -f > valhalla/tz_world.sqlite
    valhalla_build_landmarks -c valhalla/valhalla.json vietnam-latest.osm.pbf
    valhalla_build_admins -c valhalla/valhalla.json vietnam-latest.osm.pbf
    valhalla_build_tiles -c valhalla/valhalla.json vietnam-latest.osm.pbf
    valhalla_build_elevation -t -f -c valhalla/valhalla.json
    valhalla_build_tiles -c valhalla/valhalla.json vietnam-latest.osm.pbf
    valhalla_build_extract -c valhalla/valhalla.json
    valhalla_service valhalla/valhalla.json 1
