#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# see https://pypi.org/project/ruff/#history
readonly RUFF_VERSION=0.14.1
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
  python3 -m venv .venv
  source .venv/bin/activate

  local deps="ruff==$RUFF_VERSION clang-format==$CLANG_FORMAT_VERSION clang-tidy==$CLANG_TIDY_VERSION"

  local py=$1
  if [[ $(${py} -m pip list | grep -c "ruff\|clang-format\|clang-tidy") -ne 4 ]]; then
    ${py} -m pip install ${deps}
  fi
}
