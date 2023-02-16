#!/usr/bin/env bash
# Script for shared dependencies

set -x -o errexit -o pipefail -o nounset

# Adds the primeserver ppa
apt-get update --assume-yes
apt-get install --assume-yes software-properties-common
add-apt-repository ppa:valhalla-core/valhalla

readonly primeserver_version=0.7.0

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
    libgeos++-dev \
    libgeos-dev \
    libluajit-5.1-dev \
    liblz4-dev \
    libprime-server${primeserver_version} \
    libprime-server${primeserver_version}-dev \
    libspatialite-dev \
    libsqlite3-dev \
    libsqlite3-mod-spatialite \
    libtool \
    lld \
    locales \
    luajit \
    make \
    osmium-tool \
    parallel \
    pkg-config \
    prime-server${primeserver_version}-bin \
    python3-all-dev \
    python3-shapely \
    python3-pip \
    spatialite-bin \
    unzip \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*

# install protobuf from source to get a newer version than in the apt-repos
git clone https://github.com/protocolbuffers/protobuf.git \
  && cd protobuf \
  && git checkout 24487dd104 \
  && git submodule update --init --recursive \
  && ./autogen.sh \
  && ./configure \
  && make -j$(nproc) \
  && make install \
  && ldconfig

# for boost
python3 -m pip install --upgrade conan requests
