# map_matching_plus

`map_matching_plus` is a C++ library/service that matches a sequence
of locations (usually noisy e.g. GPS trajectory) to the underlying
road network. It's built upon the awesome
[Valhalla](https://github.com/valhalla) tiled data.

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
[tutorial](https://github.com/mapillary/map_matching_plus/blob/master/docs/run_service_in_docker.md)
to get the service run in Docker.

## Documentation

1. `TODO` Service API
2. `TODO` Library API
3. `TODO` Configuration
4. `TODO` The Algorithms
5. `TODO` Extending/Developing

## License

`map_matching_plus` is under BSD license. See `LICENSE` file for full
license text.
