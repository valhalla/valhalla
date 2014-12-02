# Valhalla Library #

## Motivation ##

Valhalla was created to serve as a starting point for an autotools project for anyone who might not have experience building one from scratch. It comes with a testing harness and coverage built in and requires a bunch of common dependencies for convenience.

## Building ##

Valhalla does have several dependencies that you must have before configure will properly produce a Makefile. It is true that many of these dependencies are not actually needed to build the valhalla code however we left the dependencies in because they seemed fairly common in most moderatly complex projects. If your project doesn't need a given dependency, delete it from the `m4` directory and update `configure.ac`.

## Testing ##

Tests can be found in the `tests/` directory, and are run by autotools when you run `make check`. If you are preparing a pull request or patch, please make sure these pass before submitting it.

There is a test coverage tool built into the build system, but it's fairly annoying to use. Please see
[the coverage docs](test_coverage.md) for more information and details on running it.

## Components ##

### Util ###

The Valhalla library comes with some examples of standard utilities that may or may not be applicable for a given project. They are mostly just intented as an example of what you might use to build a library within autotools. 

## Tests ##
Valhalla comes with a test suite, run with ``make check``. If you find a platform where the tests aren't passing [open an issue](https://github.com/mapzen/valhalla/issues/new).
