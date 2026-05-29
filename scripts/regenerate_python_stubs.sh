#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# Regenerate the committed .pyi stubs for the Python bindings by rebuilding
# the `*_stub` targets. Run automatically by CI when a binding source file
# changes.
#
# Build dir defaults to `build`; override with the BUILD_DIR env var or first
# positional arg, e.g. `BUILD_DIR=build/Release ./scripts/regenerate_python_stubs.sh`.

BUILD_DIR="${1:-${BUILD_DIR:-build}}"

if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]] \
  || ! grep -qiE "^ENABLE_PYTHON_BINDINGS:.*=(on|true|1|yes)$" "${BUILD_DIR}/CMakeCache.txt"; then
  echo "regenerate_python_stubs: '${BUILD_DIR}' is not a CMake build dir with ENABLE_PYTHON_BINDINGS=ON." >&2
  echo "  Configure one first, e.g.:" >&2
  echo "    cmake -B ${BUILD_DIR} -DENABLE_PYTHON_BINDINGS=ON -DCMAKE_BUILD_TYPE=Release" >&2
  exit 1
fi

cmake --build "${BUILD_DIR}" --target _valhalla_stub _graph_utils_stub predicted_speeds_stub

PYI_FILES=(
  src/bindings/python/valhalla/_valhalla.pyi
  src/bindings/python/valhalla/utils/_graph_utils.pyi
  src/bindings/python/valhalla/utils/predicted_speeds.pyi
)

# Fail loudly if any committed .pyi changed (or is missing from HEAD entirely).
# `git status --porcelain` reports both modified-tracked and untracked paths.
if [[ -n "$(git status --porcelain -- "${PYI_FILES[@]}")" ]]; then
  echo "" >&2
  echo "regenerate_python_stubs: .pyi stubs are out of date with the bindings." >&2
  echo "  Review the diff below, then 'git add' the regenerated files:" >&2
  echo "" >&2
  git --no-pager diff HEAD -- "${PYI_FILES[@]}" >&2 || true
  exit 1
fi
