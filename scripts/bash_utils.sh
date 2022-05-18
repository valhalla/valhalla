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
  echo "Installing pre-commit"
  if [[ $OS = "Linux" ]] ; then
    pip install pre-commit
  elif [[ ${OS} = "Darwin" ]] ; then
      if [[ $(command -v brew) == "" ]]; then
        # Install Homebrew
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
      else
        # Install pre-commit
        brew install pre-commit
      fi
  else
    pip install pre-commit
  fi
  echo "Setting up pre-commit hooks"
  pre-commit install
}
