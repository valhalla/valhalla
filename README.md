


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

Valhalla, and all of the projects under the Valhalla organization, use the [MIT License](COPYING).  Avatar/logo by [Jordan](https://www.instagram.com/jaykaydraws/). 

OpenStreetMap data in the `./test/data` is licensed under [ODbL](https://opendatacommons.org/licenses/odbl/) and [copyrighted](https://www.openstreetmap.org/copyright) by OSM contributors. Additional information on licenses and other requirements concerning the data sources most frequently used by Valhalla can be found in [the docs](https://valhalla.github.io/valhalla/mjolnir/data_sources/).

## Overview

There are several key features that we hope can differentiate the Valhalla project from other routing and network analysis engines. They are:

- Open source software, on open source data with a very liberal license. Should allow for transparency in development, encourage contribution and community input, and foster use in other projects.
- Tiled hierarchical data structure. Should allow users to have a small memory footprint on memory constrained devices, enable offline routing, provide a means for regional extracts and partial updates.
- Dynamic, runtime costing of edges and vertices within the graph via a plugin architecture. Should allow for customization and alternate route generation.
- C++ based API. Should allow for cross compilation of the various pieces to enable routing on offline portable devices.
- A plugin based narrative and manoeuvre generation architecture. Should allow for generation that is customized either to the administrative area or to the target locale.
- Multi-modal and time-based routes. Should allow for mixing auto, pedestrian, bike and public transportation in the same route or setting a time by which one must arrive at a location.

## Demo Server

[FOSSGIS e.V.](https://fossgis.de) hosts a demo server which is open to the public and includes a full planet graph with an [open-source web app](https://github.com/gis-ops/valhalla-app) on https://valhalla.openstreetmap.de. The HTTP API is accessible on a slightly different subdomain, e.g. https://valhalla1.openstreetmap.de/isochrone. Usage of the demo server follows the usual fair-usage policy as OSRM & Nominatim demo servers (somewhat enforced by [rate limits](https://github.com/valhalla/valhalla/discussions/3373#discussioncomment-1644713)).

## Platform Compatibility

Valhalla is fully functional on many Linux and Mac OS distributions, and is also used on iOS and Android devices.

For Windows, not all functionality is fully supported yet. Building the Valhalla library works flawlessly, as well as the following application modules:

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

Documentation is stored in the `docs/` folder in this GitHub repository. It can be viewed at [valhalla.github.io/valhalla](https://valhalla.github.io/valhalla).

## Installation

To run Valhalla locally or your own server, we recommend using our Docker image. Checkout our docker image here: https://github.com/orgs/valhalla/packages. Also, there's a [community Docker image](https://github.com/gis-ops/docker-valhalla) with more "magic" than the native one.

If you want to build Valhalla from source, follow the [documentation](https://valhalla.github.io/valhalla/building/).

For more information on binaries, see [Command Line Tools](#command-line-tools) section below and the [docs](https://valhalla.github.io/valhalla).

## Contributing

We :heart: contributions to Valhalla. They could be non-technical, e.g. translations into other languages via [Transifex](https://www.transifex.com/valhalla/valhalla-phrases/locales-en-us-json--transifex/) or documentation improvements, or technical ones like bug fixes or feature implementations. It's important to open an issue before setting out to work on a PR.

Ideally, get familiar with our [Contribution guidelines](https://github.com/valhalla/valhalla/blob/master/CONTRIBUTING.md) first.

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
valhalla_service valhalla.json isochrone '{"locations":[{"lat":42.552448,"lon":1.564865}],"costing":"auto","contours":[{"time":10,"color":"ff0000"}], "show_locations":true}'
# Alternatively you can pass a file with the same contents
valhalla_service valhalla.json isochrone isochrone_request.txt
```

It's important to note that all Valhalla logs for one-shot mode are piped to `stderr` while the actual JSON response will be in `stdout`. To completely silence the logs, pass `type: ""` to `midgard.logging` in the config file.


### Batch Script Tool

- [Batch Run_Route](https://github.com/valhalla/valhalla/blob/master/run_route_scripts/README.md)

## Related projects

The following projects are open-source and built with the intention to make it easier to use Valhalla and its features:

- [**OpenStreetMapSpeeds**](https://github.com/OpenStreetMapSpeeds/): A project conflating open GPS data to improve Valhalla's speed classification. The current JSON is from early 2022 and can be downloaded [here](https://raw.githubusercontent.com/OpenStreetMapSpeeds/schema/master/default_speeds.json) and used by setting the path in the `mjolnir.default_speeds_config` config option.
- [**docker-valhalla**](https://github.com/gis-ops/docker-valhalla): An easy-to-use, relatively magical Docker image for Valhalla, which only requires setting a few environment variables in `docker-compose.yml` to get a full-featured Valhalla instance.
- [**valhalla-operator**](https://github.com/itayankri/valhalla-operator): A k8s operator to deploy and manage Valhalla.
- [**valhalla-app**](https://github.com/gis-ops/valhalla-app): A React based web app for Valhalla, powering https://valhalla.openstreetmap.de/.
- [**valhalla-qgis-plugin**](https://github.com/gis-ops/valhalla-qgis-plugin): A QGIS plugin for Valhalla, also available in the [official QGIS plugin store](https://plugins.qgis.org/plugins/valhalla/). **Note**, it's almost deprecated and will be replaced with a much superior alternative.
- [**routingpy**](https://github.com/gis-ops/routingpy): A Python client for most open-source routing engines, including Valhalla, with a common interface for all engines. Available on [PyPI](https://pypi.org/project/routingpy/).
- [**routingjs**](https://github.com/gis-ops/routingjs): A TypeScript client for most open-source routing engines, including Valhalla, with a common interface for all engines. Available as engine-specific packages on [npm](https://www.npmjs.com/package/@routingjs/valhalla).
- [**pyvalhalla**](https://github.com/gis-ops/pyvalhalla): Python bindings for Valhalla, so its APIs can be used from within Python without a HTTP service. Available on [PyPI](https://pypi.org/project/pyvalhalla/).
- [**Valhalla_jll.jl**](https://github.com/JuliaBinaryWrappers/Valhalla_jll.jl): Valhalla binaries shipped for Julia.
