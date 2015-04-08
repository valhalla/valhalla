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

Valhalla is an open source routing engine and accompanying libraries for use with Open Street Map data. This library, Loki, can be used to associate location information to an underlying graph tile object for use in creating input to the [routing engine](https://github.com/valhalla/thor). In keeping with the Norse mythological theme, the name [Loki](http://en.wikipedia.org/wiki/Loki) was chosen as a play on the word locate. Since the library deals mostly with correlating some input (minimally a lat,lon) to an object within a graph tile, this seemed like a fitting name!

Build Status
------------

[![Circle CI](https://circleci.com/gh/valhalla/loki.svg?style=svg)](https://circleci.com/gh/valhalla/loki)

Building
--------

Loki uses the [GNU Build System](http://www.gnu.org/software/automake/manual/html_node/GNU-Build-System.html) to configure and build itself. To install on a Debian or Ubuntu system you need to get its dependencies with:

    sudo apt-get install -y autoconf automake libtool make gcc-4.8 g++-4.8 libboost1.54-dev libboost-program-options1.54-dev libboost-filesystem1.54-dev libboost-system1.54-dev protobuf-compiler libprotobuf-dev lua5.2 liblua5.2-dev

Then you should be able to bootstrap the build system:

    ./autogen.sh

And then run the standard GNU build install:

    ./configure && make && make install

Please see `./configure --help` for more options on how to control the build process.

Using
-----

For detailed information about what algorithms, data structures and executables are contained within loki, please see the more [detailed documentation](docs/index.md).

The build will produce both libraries and headers for use in other Valhalla organization projects, however you are free to use Loki for your own projects as well. To simplify the inclusion of the Loki library in another autotoolized project you may make use of [loki m4](m4/valhalla_loki.m4) in your own `configure.ac` file. For an exmample of this please have a look at `configure.ac` in another one of the valhalla projects. Loki, and all of the projects under the Valhalla organization use the [MIT License](COPYING).

Contributing
------------

We welcome contributions to loki. If you would like to report an issue, or even better fix an existing one, please use the [loki issue tracker](https://github.com/valhalla/loki/issues) on GitHub.

If you would like to make an improvement to the code, please be aware that all valhalla projects are written mostly in C++11, in the K&R (1TBS variant) with two spaces as indentation. We generally follow this [c++ style guide](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html). We welcome contributions as pull requests to the [repository](https://github.com/valhalla/loki) and highly recommend that your pull request include a test to validate the addition/change of functionality.

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
