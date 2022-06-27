#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

function setup_mason {

  readonly OS=$(uname)
  if [[ $OS = "Linux" ]] ; then
      readonly NPROC=$(nproc)
  elif [[ ${OS} = "Darwin" ]] ; then
      readonly NPROC=$(sysctl -n hw.physicalcpu)
  else
      readonly NPROC=1
  fi

  if [ ! -f mason/mason ] ; then
      echo "Installing mason"
      mkdir -p ./mason
      curl -sSfL https://github.com/mapbox/mason/archive/88f931cf8a327e3ab4802272e47a4d192f846fda.tar.gz \
        | tar \
          --gunzip \
            --extract \
            --strip-components=1 \
            --exclude="*md" \
            --exclude="test*" \
            --directory=./mason
  fi

}

function setup_pre_commit {
  echo "INFO: Installing pre-commit"
  if [[ $(command -v python3) != "" ]]; then 
    python3 -m pip install pre-commit
  elif [[ $(command -v python) != "" ]]; then
    python -m pip install pre-commit
  else
    echo "WARNING: install python3 to set up pre-commit hooks."
    return
  fi
  echo "INFO: Setting up pre-commit hooks"
  pre-commit install
}
