#!/usr/bin/env bash
set -o errexit -o pipefail -o nounset

BUILD_DIR="${1:-build_manylinux}"
# if someone specifies the python version, we'll build this wheel to ./wheelhouse
PYTHON_VERSION="${2:-}"

echo "[INFO] ccache dir is $(ccache -k cache_dir) with CCACHE_DIR=${CCACHE_DIR:-}"
# we have a few packages installed to the system as well as built from source
# we need to prioritize the ones built from source
# NOTE: this shouldn't break those packages, as the ones we duplicate so far (e.g. geos)
# have no own dependencies which might be installed on the system and thus outdated
export LD_LIBRARY_PATH="/usr/local/lib64:/usr/local/lib"

cmake -B ${BUILD_DIR} -G Ninja \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF `# turns off lto, which triggers a gcc/ld bug` \
  -DENABLE_PYTHON_BINDINGS=OFF `# setuptools will build the bindings` \
  -DENABLE_TESTS=OFF \
  -DENABLE_SINGLE_FILES_WERROR=OFF \
  -DENABLE_GDAL=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DVALHALLA_VERSION_MODIFIER=${VALHALLA_VERSION_MODIFIER:-}

echo "[INFO] Building & installing libvalhalla..."
LDFLAGS=-fno-lto cmake --build ${BUILD_DIR} -j$(nproc) > /dev/null
cmake --install ${BUILD_DIR}

ccache -s

if ! [ -z "$PYTHON_VERSION" ]; then
  echo "[INFO] Building the pyvalhalla wheel for python v${PYTHON_VERSION}."
  python$PYTHON_VERSION -m pip install -r src/bindings/python/requirements-build.txt > /dev/null
  # patch auditwheel so that it allows to vendor libraries also linked to libpython
  python$PYTHON_VERSION src/bindings/python/scripts/auditwheel_patch.py libexpat.so.1 libz.so.1
  python$PYTHON_VERSION setup.py bdist_wheel

  # repair the wheel
  for whl in dist/*; do
    python$PYTHON_VERSION -m auditwheel repair --plat manylinux_2_28_x86_64 "${whl}"
  done
fi

echo "[INFO] Done."
