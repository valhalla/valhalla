# map_matching_plus

`map_matching_plus` is a C++ library for associating a sequence of
locations (e.g. GPS trajectory) to the underlying road network. The
matching considers both road network topology and spatial
relations. The library is based on the awesome
[Valhalla](https://github.com/valhalla) routing engine.

## Building

### GNU build system
`map_matching_plus ` uses
[GNU Build System](http://www.gnu.org/software/automake/manual/html_node/GNU-Build-System.html)
to configure and build itself. To install on a Debian or Ubuntu system
you need to install its dependencies with:

    $ ./scripts/dependencies.sh

And then run to install it:

    $ ./scripts/install.sh

Please see `./configure --help` for more options on how to control the
build process.

### Docker

`map_matching_plus` also provides a
[Dockerfile](https://github.com/mapillary/map_matching_plus/blob/master/docker/Dockerfile)
to build Docker image:

    $ cd docker
    $ docker build -t mapillary/mmp .

## Getting Started

See [Getting Started](https://github.com/mapillary/map_matching_plus) for
more information.


## License

`map_matching_plus` is under BSD license. See `LICENSE` file for full
license text.
