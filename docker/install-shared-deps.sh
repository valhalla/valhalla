#!/usr/bin/env bash
# Script for shared dependencies

set -o errexit -o pipefail -o nounset

apt-get update -y && apt-get install -y software-properties-common
add-apt-repository -y ppa:valhalla-core/valhalla && apt-get update -y

apt-get install -y \
    autoconf \
    automake \
    ccache \
    clang-5.0 \
    clang-tidy-5.0 \
    coreutils \
    curl \
    g++ \
    gcc \
    git \
    jq \
    lcov \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libgeos++-dev \
    libgeos-dev \
    libluajit-5.1-dev \
    liblz4-dev \
    libprime-server0.6.3-dev \
    libprotobuf-dev \
    libspatialite-dev \
    libsqlite3-dev \
    libsqlite3-mod-spatialite \
    libtool \
    locales \
    luajit \
    make \
    ninja-build \
    osmium-tool \
    parallel \
    pkg-config \
    prime-server0.6.3-bin \
    protobuf-compiler \
    python-all-dev \
    python-minimal \
    python3-all-dev \
    python3-minimal \
    spatialite-bin \
    unzip \
    vim-common \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*

# install node
mkdir -p ${NVM_DIR}
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash
. $NVM_DIR/nvm.sh \
    && nvm install $NODE_VERSION \
    && nvm alias default $NODE_VERSION \
    && nvm use default

# install cmake
curl https://cmake.org/files/v3.16/cmake-3.16.0-Linux-$(uname --machine).sh > /tmp/cmake.sh
sh /tmp/cmake.sh --prefix=/usr/local --skip-license && /bin/rm /tmp/cmake.sh
cmake --version
