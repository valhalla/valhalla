## Valhalla Python bindings

[![pyvalhalla version](https://img.shields.io/pypi/v/pyvalhalla?label=pyvalhalla)](https://pypi.org/project/pyvalhalla/) [![pyvalhalla-weekly version](https://img.shields.io/pypi/v/pyvalhalla-weekly?label=pyvalhalla-weekly)](https://pypi.org/project/pyvalhalla-weekly/)

This folder contains the Python bindings to [Valhalla routing engine](https://github.com/valhalla/valhalla).

> [!NOTE]
> `pyvalhalla(-weekly)` packages are currently only published for:
> - `linux-x86_x64`
> - `linux-aarch64`
> - `win-amd64`
> - `macos-arm64`

On top of the (very) high-level Python bindings, we package some data-building Valhalla executables to ease the process of graph creation or run Valhalla as a service, see [below](#valhalla-executables).

### Installation

We publish CPython packages as **binary wheels** for Win (`amd64`), MacOS (`arm64`) and Linux (`x86_64`/`aarch64`) distributions with `glibc>=2.28`. To decrease disk footprint of the PyPI releases, we only publish a single `abi3` wheel per platform, which **requires Python >= 3.12**. To install on Python < 3.12, make sure to install the system dependencies as described in [the docs](https://valhalla.github.io/valhalla/building/#platform-specific-builds) before trying a `pip install pyvalhalla`.

`pip install pyvalhalla` to install the most recent Valhalla **release**.  
`pip install pyvalhalla-weekly` to install the weekly published Valhalla **master commit**.

Or manually in the current Python environment with e.g.

```shell
git clone https://github.com/valhalla/valhalla
cd valhalla
pip install .
```


In case you need to do a source installation (from `sdist`), follow the [build instructions](https://valhalla.github.io/valhalla/building/) for your platform to install the needed dependencies. Then a simple `pip install pyvalhalla` should work fine for Linux/OSX. On Windows one needs to install C++ developer tools, see also below in the developer notes for external `vcpkg` usage to resolve dependencies.

> [!TIP]
> **For developers**: `pip install -e` (editable build) will by default build into a temp directory, so everytime it's invoked it'll rebuild all of libvalhalla. Use the following command to enable real incremental builds:
> 
> ```shell
> pip install -e . --no-build-isolation \
>   -Cbuild-dir=build_python (or other build dir) \
>   -Ccmake.build-type=Release \
>   -Ccmake.define.VALHALLA_VERSION_MODIFIER="$(git rev-parse --short HEAD)"
>   # optionally for vcpkg package management
>   -Ccmake.define.CMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
>   -Ccmake.define.VCPKG_TARGET_TRIPLET=x64-windows
>   -Ccmake.define.VCPKG_OVERLAY_PORTS=overlay-ports-vcpkg
> ```
> 
> Similarly for building a wheel:
> 
> ```shell
> pip wheel . -w dist --no-build-isolation \
>   -Cbuild-dir=build_python (or other build dir) \
>   -Ccmake.build-type=Release \
>   -Ccmake.define.VALHALLA_VERSION_MODIFIER="$(git rev-parse --short HEAD)"
>   # optionally for vcpkg package management
>   -Ccmake.define.CMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
>   -Ccmake.define.VCPKG_TARGET_TRIPLET=x64-windows
>   -Ccmake.define.VCPKG_OVERLAY_PORTS=overlay-ports-vcpkg
> ```
>
> Both commands have to repeated for each build.

### Usage

#### Bindings

Find a more extended notebook in `./examples`, e.g. how to [use the actor](https://github.com/valhalla/valhalla/blob/master/src/bindings/python/examples/actor_examples.ipynb).

Before using the Python bindings you need to have access to a routable Valhalla graph. Once you installed the `pyvalhalla` package you can create one with

```shell
wget https://download.geofabrik.de/europe/andorra-latest.osm.pbf
python -m valhalla valhalla_build_tiles -c <valhalla.json> andorra-latest.osm.pbf
```

Once you have created a graph locally, you can use it like this:

```python
from valhalla import Actor, get_config, get_help

# generate configuration
config = get_config(tile_extract='./custom_files/valhalla_tiles.tar', verbose=True)

# print the help for specific config items (has the same structure as the output of get_config()
print(get_help()["service_limits"]["auto"]["max_distance"])

# instantiate Actor to load graph and call actions
actor = Actor(config)
route = actor.route({"locations": [...]})
```

#### Valhalla executables

To access the C++ (native) executables, there are 2 options:

- (recommended) execute the module, e.g. `python -m valhalla valhalla_build_tiles -h`
- execute the Python wrapper scripts directly, e.g. `valhalla_build_tiles -h`

> [!NOTE]
> For the latter option to work, the Python environment's `bin/` folder has to be in the `$PATH`. Inside virtual environments, that's always the case.

Executing the scripts directly might also not work properly if there's a system-wide Valhalla installation, unless the Python environment's `bin/` folder has higher priority than system folders in `$PATH`. The module execution uses an explicit Python executable which should be preferred.

There are also some additional commands we added:

- `--help`: print the help for `python -m valhalla` explicitly
- `--quiet`: redirect `stdout` of the C++ executables to `/dev/null`; can be added **once** anywhere in the command, will not be forwarded to a C++ executable
- `print_bin_path`: simply prints the absolute path to the package-internal `bin/` directory where the C++ executables are; useful if the executables should be accessed directly in some script

To find out which Valhalla executables are currently included, run `python -m valhalla --help`. We limit the number of executables to control the wheel size. However, we're open to include any other executable if there's a good reason.

### Building from source

Note, building the bindings from source is usually best done by building Valhalla with `cmake -B build -DENABLE_PYTHON_BINDING=ON ...`. However, if you want to package your own `pyvalhalla` bindings for some reason (e.g. fork in a bigger team), you can follow the below instructions, which are also executed by our CI.

The Python build respects a few CMake configuration variables:

- `VALHALLA_VERSION_MODIFIER` (optional): Will append a string to the actual Valhalla version string, e.g. `$(git rev-parse --short HEAD)` will append the current branch's commit hash.

#### `cibuildwheel`

On our CI, this orchestrates the packaging of all `pyvalhalla` wheels for every supported, minor Python version and every platform. It can also be run locally (obviously only being able to build wheels for _your_ platform), e.g.

```shell
python -m pip install cibuildwheel
cibuildwheel --print-build-identifiers
cibuildwheel --only cp313-manylinux_x86_64

# for windows you'll have to set an env var to the vcpkg win root
VCPKG_ARCH_ROOT="build/vcpkg_installed/custom-x64-windows" cibuildwheel --only cp313-win_amd64
```

The build looks at a few environment variables:

- `VCPKG_ARCH_ROOT` (required for Win): The relative/absolute directory of the `vcpkg` root.

In the end, you'll find the wheel in `./wheelhouse`.

#### Linux

To package arch-dependent Linux bindings we use a `manylinux` fork, where we install all dependencies into the `manylinux_2_28` image, based on AlmaLinux 8. This is necessary to have a broad `glibc` compatibility with many semi-recent Linux distros.

Either pull the `manylinux` image, or build it locally for testing:

```shell
docker pull ghcr.io/valhalla/manylinux:2_28_valhalla_python

# or pull the image from ghcr.io
git clone https://github.com/valhalla/manylinux
cd manylinux
POLICY=manylinux_2_28 PLATFORM=x86_64 COMMIT_SHA=$(git rev-parse --verify HEAD) BUILDX_BUILDER=builder-docker-container ./build.sh
docker tag quay.io/pypa/manylinux_2_28_x86_64:$(git rev-parse --verify HEAD) ghcr.io/valhalla/manylinux:2_28_valhalla_python
```

Once built, start a container to actually build Valhalla using AlmaLinux 8:

```shell
cd valhalla
docker run -dt -v $PWD:/valhalla-py --name valhalla-py --workdir /valhalla-py ghcr.io/valhalla/manylinux:2_28_valhalla_python
docker exec -t valhalla-py /valhalla-py/src/bindings/python/scripts/build_manylinux.sh build_manylinux 3.13
```

This will also build & install `libvalhalla` before building the bindings. At this point there should be a `wheelhouse` folder with the fixed python wheel, ready to be installed or distributed to arbitrary python 3.13 installations.

### Testing (**`linux` only**)

We have a small [test script](https://github.com/valhalla/valhalla/blob/master/src/bindings/python/test/test_pyvalhalla_package.sh) which makes sure that all the executables are working properly. If run locally for some reason, install a `pyvalhalla` wheel first. We run this in CI in a fresh Docker container with no dependencies installed, mostly to verify dynamic linking of the vendored dependencies.
