#!/usr/bin/env bash

# the test assumes that pyvalhalla(-weekly) is installed to the current "python" installation
# it mostly makes sure there are no linked libraries missing

set -x -o errexit -o pipefail -o nounset

python -m valhalla --help | grep "Available commands"

bin_path=$(python -m valhalla print_bin_path)
binaries=$(find "$bin_path" -type f)
if [ -z "$binaries" ]; then
    echo "[FATAL] found no binaries at $bin_path"
    exit 1
fi

for binary_path in $binaries; do
    binary=$(basename "$binary_path")
    python -m valhalla "$binary" -h
    python_wrapper=$(which "$binary")
    file $python_wrapper | grep -F "Python script" || (echo "$python_wrapper is not a Python script" && exit 1)
    $binary -h
done

echo "[INFO] Success!"
