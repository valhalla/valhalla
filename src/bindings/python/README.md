## Python bindings

> [!NOTE]
> `pyvalhalla(-git)` packages are currently only published for `linux-x64` and Linux

### Linux

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

On our CI, this orchestrates the build of all `pyvalhalla` wheels for every supported, minor Python version and every platform. It can also be run locally (obviously only being able to build wheels for _your_ platform), e.g.

```shell
# specifiers from https://cibuildwheel.pypa.io/en/stable/options/#build-skip
cibuildwheel --only cp313-manylinux_x86_64
```

This might download the `manylinux` docker image. In the end, you'll find the wheel in `./wheelhouse`.
