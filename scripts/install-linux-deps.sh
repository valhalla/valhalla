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
    libprotobuf-dev \
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
    protobuf-compiler \
    python3-all-dev \
    python3-shapely \
    python3-pip \
    spatialite-bin \
    unzip \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*

# for boost
python3 -m pip install --upgrade conan requests
