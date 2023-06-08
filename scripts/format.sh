#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# Runs the Clang Formatter in parallel on the code base.
# Return codes:
#  - 1 there are files to be formatted
#  - 0 everything looks fine


readonly CLANG_FORMAT_VERSION=11.0.0

source scripts/bash_utils.sh
setup_mason

./mason/mason install clang-format $CLANG_FORMAT_VERSION
./mason/mason link clang-format $CLANG_FORMAT_VERSION
readonly CLANG_FORMAT=$(pwd)/mason_packages/.link/bin/clang-format

echo "Using clang-format $CLANG_FORMAT_VERSION from ${CLANG_FORMAT}"

find src valhalla test bench -type f -name '*.h' -o -name '*.cc' \
  | xargs -I{} -P ${NPROC} ${CLANG_FORMAT} -i -style=file {}


# Python setup
py=$(setup_python)
${py} -m pip install black==22.10.0 flake8==5.0.4
python_sources=$(LANG=C find scripts src/bindings/python -type f -exec file {} \; | grep -F "Python script" | sed 's/:.*//')

# Python formatter
${py} -m black --line-length=105 --skip-string-normalization ${python_sources}

# Python linter
${py} -m flake8 --max-line-length=105 --extend-ignore=E501,E731,E203 --extend-exclude=src/bindings/python/__init__.py ${python_sources}
