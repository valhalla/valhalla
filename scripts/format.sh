#!/usr/bin/env bash

set -o errexit
set -o pipefail
set -o nounset

# Runs the Clang Formatter in parallel on the code base.
# Return codes:
#  - 1 there are files to be formatted
#  - 0 everything looks fine

# Get CPU count
OS=$(uname)
NPROC=1
if [[ $OS = "Linux" ]] ; then
    NPROC=$(nproc)
elif [[ ${OS} = "Darwin" ]] ; then
    NPROC=$(sysctl -n hw.physicalcpu)
fi

# Discover clang-format
if type clang-format-3.8 2> /dev/null ; then
    CLANG_FORMAT=clang-format-3.8
elif type clang-format 2> /dev/null ; then
    # Clang format found, but need to check version
    CLANG_FORMAT=clang-format
    V=$(clang-format --version)
    if [[ $V != *3.8* ]] ; then
        echo "Installed clang-format is not version 3.8"
        if [ ! -f $(pwd)/mason_packages/.link/bin/clang-format ] ; then
            echo "Installing clang-format 3.8 via mason"
            mkdir ./mason
            curl -sSfL https://github.com/mapbox/mason/archive/v0.18.0.tar.gz | tar --gunzip --extract --strip-components=1 --exclude="*md" --exclude="test*" --directory=./mason
            ./mason/mason install clang-format 3.8.1
            ./mason/mason link clang-format 3.8.1
        fi
        echo "Using clang-format 3.8 from $(pwd)/mason_packages/.link/bin"
        PATH="$(pwd)/mason_packages/.link/bin:$PATH"
        #exit 1
    fi
else
    echo "No clang-format found"
    if [ ! -f $(pwd)/mason_packages/.link/bin/clang-format ] ; then
        echo "Installing clang-format 3.8 via mason"
        mkdir ./mason
        curl -sSfL https://github.com/mapbox/mason/archive/v0.18.0.tar.gz | tar --gunzip --extract --strip-components=1 --exclude="*md" --exclude="test*" --directory=./mason
        ./mason/mason install clang-format 3.8.1
        ./mason/mason link clang-format 3.8.1
    fi
    echo "Using clang-format 3.8 from $(pwd)/mason_packages/.link/bin"
    CLANG_FORMAT=clang-format
    PATH="$(pwd)/mason_packages/.link/bin:$PATH"
fi

find src valhalla test -type f -name '*.h' -o -name '*.cc' \
  | xargs -I{} -P ${NPROC} ${CLANG_FORMAT} -i -style=file {}
