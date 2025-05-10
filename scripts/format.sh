#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# Runs the Clang Formatter in parallel on the code base.
# Return codes:
#  - 1 there are files to be formatted
#  - 0 everything looks fine

if [[ $(uname -i) == 'aarch64' ]]; then
  echo 'Formatting is disabled on arm for the time being'
  exit
fi
source scripts/bash_utils.sh

# Python setup
py=$(setup_python)
install_py_packages $py
python_sources=$(LANG=C find scripts src/bindings/python -type f -exec file {} \; | grep -F "Python script" | sed 's/:.*//')

# Python formatter
${py} -m black --line-length=105 --skip-string-normalization ${python_sources}

# Python linter
${py} -m flake8 --max-line-length=105 --extend-ignore=E501,E731,E203 --extend-exclude=src/bindings/python/__init__.py ${python_sources}

# clang-format
echo "Using $(${py} scripts/clang_format_wrapper.py --version)"

# determine how many threads to use
readonly OS=$(uname)
if [[ $OS = "Linux" ]] ; then
    readonly NPROC=$(nproc)
elif [[ ${OS} = "Darwin" ]] ; then
    readonly NPROC=$(sysctl -n hw.physicalcpu)
else
    readonly NPROC=1
fi

find src valhalla test -type f -name '*.h' -o -name '*.cc' \
  | xargs -I{} -P ${NPROC} ${py} scripts/clang_format_wrapper.py -style=file -i {}
