#!/usr/bin/env bash
# Script for shared dependencies

set -x -o errexit -o pipefail -o nounset

# Now, go through and install the build dependencies
sudo apt-get update --assume-yes
env DEBIAN_FRONTEND=noninteractive sudo apt install --yes --quiet \
    autoconf \
    automake \
    ccache \
    clang \
    clang-tidy \
    coreutils \
    curl \
    cmake \
    g++ \
    gcc \
    git \
    jq \
    lcov \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libczmq-dev \
    libgdal-dev \
    libgeos++-dev \
    libgeos-dev \
    libluajit-5.1-dev \
    liblz4-dev \
    libprotobuf-dev \
    libspatialite-dev \
    libsqlite3-dev \
    libsqlite3-mod-spatialite \
    libtool \
    libzmq3-dev \
    lld \
    locales \
    luajit \
    make \
    osmium-tool \
    parallel \
    pkgconf \
    protobuf-compiler \
    python3-all-dev \
    python3-shapely \
    python3-requests \
    python3-pip \
    spatialite-bin \
    unzip \
    zlib1g-dev
  
# build prime_server from source
# readonly primeserver_version=0.7.0
readonly primeserver_dir=/tmp/prime_server
git clone --recurse-submodules https://github.com/kevinkreiser/prime_server $primeserver_dir
pushd $primeserver_dir
./autogen.sh && ./configure
make -j${CONCURRENCY:-$(nproc)}
sudo make install
popd && rm -rf $primeserver_dir
