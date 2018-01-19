#!/bin/bash
set -e

# get some dependencies
sudo add-apt-repository -y ppa:valhalla-core/valhalla
sudo apt-get update
sudo apt-get install -y autoconf automake make libtool pkg-config g++ gcc jq lcov protobuf-compiler vim-common libboost1.54-all-dev libboost-all-dev libcurl4-openssl-dev libgeos-dev libgeos++-dev liblua5.2-dev libprime-server0.6.3-dev libprotobuf-dev libspatialite-dev libsqlite3-dev libsqlite3-mod-spatial python-all-dev zlib1g-dev liblz4-dev lua5.2 prime-server0.6.3-bin ccache

# test some easier stuff
for f in *.json; do
  python -c "import json; json.load(open('$f'))"
done
for f in lua/*.lua; do
  lua $f
done
for f in locales/*.json; do
  python -c "import json; json.load(open('$f'))"
done
scripts/valhalla_build_config

# setup some caching and do the build
mkdir -p ${HOME}/.ccache && echo "max_size = 4.0G" > ${HOME}/.ccache/ccache.conf
./autogen.sh
./configure CC="ccache gcc" CXX="ccache g++" --enable-coverage
ccache -z
ccache -s
make test -j$(nproc) || true
ccache -s
