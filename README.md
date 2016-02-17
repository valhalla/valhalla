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

Valhalla is an open source routing engine and accompanying libraries for use with Open Street Map data. This library, Baldr, serves as a set of routing-specific data structures distilled into a library for use within other projects under the valhalla organization. In keeping with the Norse mythological theme, the name [Baldr](http://en.wikipedia.org/wiki/Baldr) was chosen as a backronym standing for: Base ALgorithms and Data Resource. Since the library deals mostly with accessing routing data and algorithms related to routing subproblems.

Build Status
------------

[![Circle CI](https://circleci.com/gh/valhalla/baldr.svg?style=svg)](https://circleci.com/gh/valhalla/baldr)

Building
--------

Baldr uses the [GNU Build System](http://www.gnu.org/software/automake/manual/html_node/GNU-Build-System.html) to configure and build itself. To install on a Debian or Ubuntu system you need to install its dependencies with:

    srcipts/dependencies.sh

And then run to install it:

    scripts/install.sh

Please see `./configure --help` for more options on how to control the build process.

Using
-----

For detailed information about what algorithms, data structures and executables are contained within baldr, please see the more [detailed documentation](docs/index.md).

The build will produce both libraries and headers for use in other Valhalla organization projects, however you are free to use Baldr for your own projects as well. To simplify the inclusion of the Baldr library in another autotoolized project you may make use of [baldr m4](m4/valhalla_baldr.m4) in your own `configure.ac` file. For an exmample of this please have a look at `configure.ac` in another one of the valhalla projects. Baldr, and all of the projects under the Valhalla organization use the [MIT License](COPYING).

Contributing
------------

We welcome contributions to baldr. If you would like to report an issue, or even better fix an existing one, please use the [baldr issue tracker](https://github.com/valhalla/baldr/issues) on GitHub.

If you would like to make an improvement to the code, please be aware that all valhalla projects are written mostly in C++11, in the K&R (1TBS variant) with two spaces as indentation. We generally follow this [c++ style guide](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html). We welcome contributions as pull requests to the [repository](https://github.com/valhalla/baldr) and highly recommend that your pull request include a test to validate the addition/change of functionality.

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

Timezone specs
--------------

The csv file containing the zone_specs is used by the boost::local_time::tz_database and is located in the date_time directory.  We will try to periodically update the zonespec.csv file; however, if you find that it is outdated, please download the latest copy in your date_time directory and submit a pull request.  Thank you!     

Download command:
wget -q https://raw.githubusercontent.com/boostorg/date_time/master/data/date_time_zonespec.csv -O zonespec.csv

More information on the zone_specs can be found here:  https://github.com/boostorg/date_time/tree/master/data

