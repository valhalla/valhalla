#!/usr/bin/env bash
set -o errexit -o pipefail -o nounset

BUILD_DIR="${1:-build_manylinux}"
# if someone specifies the python version, we'll build this wheel to ./wheelhouse
PYTHON_VERSION="${2:-}"

cmake -B ${BUILD_DIR} \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF `# turns off lto, which triggers a gcc/ld bug` \
  -DENABLE_PYTHON_BINDINGS=OFF `# setuptools will build the bindings` \
  -DENABLE_TESTS=OFF \
  -DENABLE_SINGLE_FILES_WERROR=OFF \
  -DENABLE_GDAL=ON \
  -DCMAKE_BUILD_TYPE=Release

echo "[INFO] Building & installing libvalhalla..."
LDFLAGS=-fno-lto cmake --build ${BUILD_DIR} -- -j$(nproc) > /dev/null
make -C ${BUILD_DIR} install

# copy most recent valhalla_build_config.py
cp scripts/valhalla_build_config src/bindings/python/valhalla/valhalla_build_config.py

if ! [ -z "$PYTHON_VERSION" ]; then
  echo "[INFO] Building the pyvalhalla wheel for python v${PYTHON_VERSION}."
  python$PYTHON_VERSION -m pip install -r src/bindings/python/requirements-build.txt > /dev/null
  python$PYTHON_VERSION setup.py bdist_wheel

  # repair the wheel
  for whl in dist/*; do
    auditwheel repair --plat manylinux_2_28_x86_64 "${whl}"
  done
fi

echo "[INFO] Done."
