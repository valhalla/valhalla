#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# see https://pypi.org/project/black/#history
readonly BLACK_VERSION=24.10.0
# see https://pypi.org/project/flake8/#history
readonly FLAKE8_VERSION=7.1.1
# see https://pypi.org/project/clang-format/#history
readonly CLANG_FORMAT_VERSION=11.0.1
# see https://pypi.org/project/clang-tidy/#history
readonly CLANG_TIDY_VERSION=20.1.0

function setup_python {
  local python_bin=""
  if [[ $(command -v python3) != "" ]]; then
    python_bin="python3"
  elif [[ $(command -v python) != "" ]]; then
    python_bin="python"
  else
    echo "WARNING: install python3 for linting" 1>&2
    return
  fi
  echo ${python_bin}
}

function install_py_packages {
  local deps="black==$BLACK_VERSION flake8==$FLAKE8_VERSION clang-format==$CLANG_FORMAT_VERSION clang-tidy==$CLANG_TIDY_VERSION"

  local py=$1
  if [[ $(${py} -m pip list | grep -c "black\|flake8\|clang-format\|clang-tidy") -ne 4 ]]; then
    if [[ $(${py} -c 'import sys; print(int(sys.base_prefix != sys.prefix or hasattr(sys, "real_prefix")))') -eq 1 ]]; then
      ${py} -m pip install ${deps}
    else
      sudo PIP_BREAK_SYSTEM_PACKAGES=1 ${py} -m pip install ${deps}
    fi
  fi
}
