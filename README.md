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

Valhalla is an open source routing engine and accompanying libraries for use with OpenStreetMap data. This library, Odin, serves as a directions engine for annotating a path as input from the [routing engine](https://github.com/valhalla/thor) for use in navigation. In keeping with the Norse mythological theme, the name [Odin](http://en.wikipedia.org/wiki/Odin) was chosen as he has often been noted as being very wise. Since the library deals mostly with providing, hopefully wise, guidance along a path to be used for navigation, this seemed like a fitting name! We've also managed to create a backronym out of Odin which stands for: Open Directions and Improved Narrative.

Build Status
------------

[![Circle CI](https://circleci.com/gh/valhalla/odin.svg?style=svg)](https://circleci.com/gh/valhalla/odin)

Building
--------

Odin uses the [GNU Build System](http://www.gnu.org/software/automake/manual/html_node/GNU-Build-System.html) to configure and build itself. To install on a Debian or Ubuntu system you need to install its dependencies with:

    srcipts/dependencies.sh

And then run to install it:

    scripts/install.sh

Please see `./configure --help` for more options on how to control the build process.

Using
-----

For detailed information about what algorithms, data structures and executables are contained within odin, please see the more [detailed documentation](docs/index.md).

The build will produce both libraries and headers for use in other Valhalla organization projects, however you are free to use Odin for your own projects as well. To simplify the inclusion of the Odin library in another autotoolized project you may make use of [odin m4](m4/valhalla_odin.m4) in your own `configure.ac` file. For an exmample of this please have a look at `configure.ac` in another one of the valhalla projects. Odin, and all of the projects under the Valhalla organization use the [MIT License](COPYING).

Contributing
------------

We welcome contributions to odin. If you would like to report an issue, or even better fix an existing one, please use the [odin issue tracker](https://github.com/valhalla/odin/issues) on GitHub.

If you would like to make an improvement to the code, please be aware that all valhalla projects are written mostly in C++11, in the K&R (1TBS variant) with two spaces as indentation. We generally follow this [c++ style guide](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html). We welcome contributions as pull requests to the [repository](https://github.com/valhalla/odin) and highly recommend that your pull request include a test to validate the addition/change of functionality.

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
