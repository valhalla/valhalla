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

Valhalla is an open source routing engine and accompanying libraries for use with Open Street Map data.

Build Status
------------

[![Circle CI](https://circleci.com/gh/valhalla/midgard.svg?style=svg)](https://circleci.com/gh/valhalla/midgard)

Building
--------

Valhalla uses the [GNU Build System](http://www.gnu.org/software/automake/manual/html_node/GNU-Build-System.html) to configure and build itself and has a number of useful dependencies already included for convenience such as: [Boost libraries](http://boost.org/), [Protobuf](https://github.com/google/protobuf/), [cURL](http://curl.haxx.se/), [ZeroMQ](http://zeromq.org/), [SQLite](http://sqlite.org/), and [Lua](http://www.lua.org/). To install on a Debian or Ubuntu system, please first install the prerequisites:

    sudo apt-get install libboost1.54-dev lcov

Then you should be able to bootstrap the build system:

    ./autogen.sh

And then run the standard GNU build install:

    ./configure && make test && make install

Please see `./configure --help` for more options on how to control the build process.

Using
-----

For convenience valhalla has a main executable, `valhalla`, that makes use of the library it builds. The purpose of this executable to serve as a starting point for building several custom executables against the various custom libraries.

Contributing
------------

We welcome contributions to valhalla. If you would like to report an issue, please use the [valhalla issue tracker](https://github.com/mapzen/valhalla/issues) on GitHub.

If you would like to make an improvement to the code, please be aware that valhalla is written mostly in C++11, in the K&R (1TBS variant) with two spaces as indentation. We generally follow this [C++ Style Guide](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html). We welcome contributions as pull requests to the [repository](https://github.com/mapzen/valhalla).

It is possible to build a test coverage report, please see [test coverage documentation](docs/test_coverage.md) for details.
