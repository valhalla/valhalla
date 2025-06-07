## Valhalla Python bindings

This package contains the Python bindings to [Valhalla routing engine](https://github.com/valhalla/valhalla).

> [!NOTE]
> `pyvalhalla(-git)` packages are currently only published for:
> - `linux-x86_x64`
> - `win-amd64`
> - `macos-arm64`

### License

Due to the dependencies we package, the Python bindings are licensed under GNU Lesser General Public License v2 or later (LGPLv2+).

### Installation

We distribute all currently maintained CPython versions as **binary wheels** for Win64, MacOS (`arm64`) and Linux (`x86_64`) distributions with `glibc>=2.28`. We **do not** offer a source distribution on PyPI.

`pip install pyvalhalla` to install the most recent Valhalla **release**.
`pip install pyvalhalla-git` to install the most recent Valhalla **master commit**.

### Usage

Find a more extended notebook in `./examples`, e.g. how to [use the actor](./examples/actor_examples.ipynb).

Before using the Python bindings you need to have access to a routable Valhalla graph. Either install Valhalla from source and built the graph from OSM compatible data or use our [Valhalla docker image](https://github.com/gis-ops/docker-valhalla) for a painless experience, e.g. this will build the routing graph for Andorra in `./custom_files`:

```shell
docker run --rm --name valhalla_gis-ops -p 8002:8002 -v $PWD/custom_files:/custom_files -e tile_urls=https://download.geofabrik.de/europe/andorra-latest.osm.pbf gisops/valhalla:latest
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

### Building from source

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
docker exec -t valhalla-py /valhalla-py/src/bindings/python/build_linux.sh build_manylinux 3.12
```

This will also build & install `libvalhalla` before building the bindings. At this point there should be a `wheelhouse` folder with the fixed python wheel, ready to be installed or distributed to arbitrary python 3.12 installations.

### `cibuildwheel`

On our CI, this orchestrates the packaging of all `pyvalhalla` wheels for every supported, minor Python version and every platform. It can also be run locally (obviously only being able to build wheels for _your_ platform), e.g.

```shell
# specifiers from https://cibuildwheel.pypa.io/en/stable/options/#build-skip
cibuildwheel --only cp313-manylinux_x86_64

# for windows you'll have to set an env var to the vcpkg win root
VCPKG_ARCH_ROOT="build/vcpkg_installed/custom-x64-windows" cibuildwheel --only cp313-manylinux_x86_64
```

On Linux, this might download the `manylinux` docker image. In the end, you'll find the wheel in `./wheelhouse`.
