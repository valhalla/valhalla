#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# Runs the Clang Formatter in parallel on the code base.
# Return codes:
#  - 1 there are files to be formatted
#  - 0 everything looks fine

# see https://pypi.org/project/black/#history
readonly BLACK_VERSION=24.10.0
# see https://pypi.org/project/flake8/#history
readonly FLAKE8_VERSION=7.1.1
# see https://pypi.org/project/clang-format/#history
readonly CLANG_FORMAT_VERSION=11.0.1

if [[ $(uname -i) == 'aarch64' ]]; then
  echo 'Formatting is disabled on arm for the time being'
  exit
fi
source scripts/bash_utils.sh

# Python setup
py=$(setup_python)
if [[ $(${py} -m pip list | grep -c "black\|flake8\|clang-format") -ne 2 ]]; then
  if [[ $(${py} -c 'import sys; print(int(sys.base_prefix != sys.prefix or hasattr(sys, "real_prefix")))') -eq 1 ]]; then
    ${py} -m pip install black==$BLACK_VERSION flake8==$FLAKE8_VERSION clang-format==$CLANG_FORMAT_VERSION
  else
    sudo PIP_BREAK_SYSTEM_PACKAGES=1 ${py} -m pip install black==$BLACK_VERSION flake8==$FLAKE8_VERSION clang-format==$CLANG_FORMAT_VERSION
  fi
fi
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
