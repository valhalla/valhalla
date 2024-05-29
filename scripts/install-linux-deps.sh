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
    python3-pip \
    spatialite-bin \
    unzip \
    zlib1g-dev
  
# build prime_server from source
cd ./prime_server
./autogen.sh
./configure
make -j${CONCURRENCY:-$(nproc)}
make install
cd -

# for boost and scripts deps
if [[ $(python3 -c 'import sys; print(int(sys.base_prefix != sys.prefix or hasattr(sys, "real_prefix")))') -eq 1 ]]; then
  python3 -m pip install --upgrade requests shapely
else
  PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install --upgrade requests shapely
fi
