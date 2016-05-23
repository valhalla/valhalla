# map_matching_plus

`map_matching_plus` is a C++ library/service that matches a sequence
of locations (usually noisy e.g. GPS trajectory) to the underlying
road network. It's built upon the awesome
[Valhalla](https://github.com/valhalla) tiled data.

![Map](docs/figures/cover.png)

## Build Status

[![CircleCI](https://circleci.com/gh/valhalla/meili.svg?style=svg)](https://circleci.com/gh/valhalla/meili)

## Building

`map_matching_plus ` uses
[GNU Build System](http://www.gnu.org/software/automake/manual/html_node/GNU-Build-System.html)
to configure and build itself. To install on a Debian or Ubuntu system
you need to install its dependencies with:

    $ ./scripts/dependencies.sh

And then run to install it:

    $ ./scripts/install.sh

Please see `./configure --help` for more options on how to control the
build process.

## Getting Started

Follow this
[tutorial](https://github.com/valhalla/meili/blob/master/docs/run_service_in_docker.md)
to get the service run in Docker.

## Documentation

1. [Service API](https://github.com/valhalla/meili/blob/master/docs/service_api.md)
2. [Library API](https://github.com/valhalla/meili/blob/master/docs/library_api.md)
3. [Configuration](https://github.com/valhalla/meili/blob/master/docs/configuration.md)
4. [The Algorithms](https://github.com/valhalla/meili/blob/master/docs/algorithms.md)

## License

`map_matching_plus` is under BSD license. See `LICENSE` file for full
license text.
