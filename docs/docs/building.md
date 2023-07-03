We aim to make Valhalla installable on every half-modern hardware, platform and architecture. If the below instructions are not working for you, we'd be happy if you [file an issue or open a PR with a fix](https://github.com/valhalla/valhalla/blob/master/CONTRIBUTING.md).

### Build Configuration (all platforms)

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

If you're building on Apple Silicon and use the Rosetta terminal (see below), you might need to additionally specify the appropriate options:

```
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES="x86_64"
```

### Building from Source - Linux

To install on a Debian or Ubuntu system you need to install its dependencies with:

```bash
./scripts/install-linux-deps.sh
```

Now you can build and install Valhalla, e.g. 

```bash
# will build to ./build
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j$(nproc)
sudo make -C build install
```

### Building from Source - macOS

#### Configuring Rosetta for ARM64 MacBook

Check your architecture typing `arch` in the terminal. In case the result is `arm64` set up Rosetta terminal to emulate x86_64 behavior. Otherwise, skip this step.

1. Go to `Finder > Application > Utilities`.
2. Select `Terminal` and right-click on it, then choose `Duplicate`.
3. Rename the duplicated app `Rosetta Terminal`.
4. Now select `Rosetta Terminal` application, right-click and choose `Get Info` .
5. Check the box for `Open using Rosetta`, then close the `Get Info` window.
6. Make shure you get `i386` after typing `arch` command in  `Rosetta Terminal`.
7. Now it fully supports Homebrew and other x86_64 command line applications.

Install [Homebrew](http://brew.sh) in the `Rosetta Terminal` app and update the aliases.

```
echo "alias ibrew='arch -x86_64 /usr/local/bin/brew'" >> ~/.zshrc
echo "alias mbrew='arch -arm64e /opt/homebrew/bin/brew'" >> ~/.zshrc
```

You will use them to specify the platform when installing a library. Note: use `ibrew` in `Rosetta Terminal` to install all dependencies for `valhalla` and `prime_server` projects.

**_NOTE:_** If when installing packages below you get message `attempting to link with file built for macOS-arm64`, you can remove already installed packages for arm64 i.e. `mbrew uninstall ...`. Also, if there are problems with individual packages, you can install them from sources e.g. [geos](https://github.com/libgeos/geos) or [sqlite](https://www.sqlite.org/download.html).

**_NOTE:_** It is possible to build Valhalla natively for Apple Silicon, but some dependencies(e.g. LuaJIT) don't have stable versions supporting Apple Silicon and have to be built and installed manually from source.

#### Installing dependencies

To install valhalla on macOS, you need to install its dependencies with [Homebrew](http://brew.sh):

```bash
# install dependencies (automake & czmq are required by prime_server)
brew install automake cmake libtool protobuf-c libspatialite pkg-config sqlite3 jq curl wget czmq lz4 spatialite-tools unzip luajit
# following packages are needed for running Linux compatible scripts
brew install bash coreutils binutils
# Update your PATH env variable to include /usr/local/opt/binutils/bin:/usr/local/opt/coreutils/libexec/gnubin
```

Now, clone the Valhalla repository

```bash
git clone --recurse-submodules https://github.com/valhalla/valhalla.git
```

Then, build [`prime_server`](https://github.com/kevinkreiser/prime_server#build-and-install).

After getting the dependencies install it with e.g.:

```bash
# will build to ./build
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j$(sysctl -n hw.physicalcpu)
sudo make -C build install
```

### Building from Source - Windows

It's recommended to work with the following toolset:
- Visual Studio with C++ support
- Visual Studio Code (easier and lighter to handle)
- [vcpkg](https://github.com/Microsoft/vcpkg) to install packages
- [CMake](https://cmake.org/download/)

1. Install the following packages with `vcpkg` and your platform triplet (e.g. `x64-windows`). Note, you can remove all packages after `zlib` in `.\.vcpkg_deps.txt` if you don't want to build `TOOLS` & `DATA_TOOLS`:
```
# Basic packages
git -C C:\path\to\vcpkg checkout f4bd6423
cd C:\path\to\project
C:\path\to\vcpkg.exe --triplet x64-windows install "@.vcpkg_deps.txt"
```
2. Let CMake configure the build with the required modules enabled. The final command for `x64` could look like
```
"C:\Program Files\CMake\bin\cmake.EXE" --no-warn-unused-cli -DENABLE_TOOLS=ON -DENABLE_DATA_TOOLS=ON -DENABLE_PYTHON_BINDINGS=ON -DENABLE_HTTP=ON -DENABLE_CCACHE=OFF -DENABLE_SERVICES=OFF -DENABLE_BENCHMARKS=OFF -DENABLE_TESTS=OFF -DVCPKG_TARGET_TRIPLET=x64-windows -DCMAKE_TOOLCHAIN_FILE=path\to\vcpkg\scripts\buildsystems\vcpkg.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -Hpath/to/project -Bpath/to/project/build -G "Visual Studio 16 2019" -T host=x64 -A x64
```
3. Run the build for all targets.
```
cd C:\path\to\project
cmake -B build --config Release -- /clp:ErrorsOnly /p:BuildInParallel=true /m:8
```

The artifacts will be built to `./build/Release`.

## Running Valhalla server on Unix

The following script should be enough to make some routing data and start a server using it. (Note - if you would like to run an elevation lookup service with Valhalla follow the instructions [here](./elevation.md)).

```bash
# download some data and make tiles out of it
# NOTE: you can feed multiple extracts into pbfgraphbuilder
wget http://download.geofabrik.de/europe/switzerland-latest.osm.pbf http://download.geofabrik.de/europe/liechtenstein-latest.osm.pbf
# get the config and setup
mkdir -p valhalla_tiles
valhalla_build_config --mjolnir-tile-dir ${PWD}/valhalla_tiles --mjolnir-tile-extract ${PWD}/valhalla_tiles.tar --mjolnir-timezone ${PWD}/valhalla_tiles/timezones.sqlite --mjolnir-admin ${PWD}/valhalla_tiles/admins.sqlite > valhalla.json
# build timezones.sqlite to support time-dependent routing
valhalla_build_timezones > valhalla_tiles/timezones.sqlite
# build routing tiles
# TODO: run valhalla_build_admins?
valhalla_build_tiles -c valhalla.json switzerland-latest.osm.pbf liechtenstein-latest.osm.pbf
# tar it up for running the server
# either run this to build a tile index for faster graph loading times
valhalla_build_extract -c valhalla.json -v
# or simply tar up the tiles
find valhalla_tiles | sort -n | tar cf valhalla_tiles.tar --no-recursion -T -

# grab the demos repo and open up the point and click routing sample
git clone --depth=1 --recurse-submodules --single-branch --branch=gh-pages https://github.com/valhalla/demos.git
firefox demos/routing/index-internal.html &
# NOTE: set the environment pulldown to 'localhost' to point it at your own server

# start up the server
valhalla_service valhalla.json 1
# curl it directly if you like:
curl http://localhost:8002/route --data '{"locations":[{"lat":47.365109,"lon":8.546824,"type":"break","city":"Zürich","state":"Altstadt"},{"lat":47.108878,"lon":8.394801,"type":"break","city":"6037 Root","state":"Untere Waldstrasse"}],"costing":"auto","directions_options":{"units":"miles"}}' | jq '.'

#HAVE FUN!
```
