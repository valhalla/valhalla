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
      curl -sSfL https://github.com/mapbox/mason/archive/6e12456fd85e842eda63e87e5b706a7e961e522d.tar.gz \
        | tar \
          --gunzip \
            --extract \
            --strip-components=1 \
            --exclude="*md" \
            --exclude="test*" \
            --directory=./mason
  fi

}
