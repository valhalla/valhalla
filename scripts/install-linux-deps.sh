#!/usr/bin/env bash
# Script for shared dependencies

set -x -o errexit -o pipefail -o nounset

# Now, go through and install the build dependencies
apt-get update --assume-yes
env DEBIAN_FRONTEND=noninteractive apt-get install --yes --quiet \
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
    libcurl4-openssl-dev \
    libczmq-dev \
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
    pkg-config \
    protobuf-compiler \
    python3-all-dev \
    python3-shapely \
    python3-pip \
    spatialite-bin \
    unzip \
    zlib1g-dev
  
# build prime_server from source
# readonly primeserver_version=0.7.0
readonly primeserver_dir=/usr/local/src/prime_server
git clone --recurse-submodules https://github.com/kevinkreiser/prime_server $primeserver_dir
pushd $primeserver_dir
./autogen.sh && ./configure && \
make -j${CONCURRENCY:-$(nproc)} install && \
popd && \
rm -r $primeserver_dir

# for boost
python3 -m pip install --upgrade "conan<2.0.0" requests shapely
