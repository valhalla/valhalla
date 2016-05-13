#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ lcov libboost1.54-all-dev protobuf-compiler libprotobuf-dev lua5.2 liblua5.2-dev libsqlite3-dev libspatialite-dev libgeos-dev libgeos++-dev libcurl4-openssl-dev

#skadi has everything
mkdir -p deps
rm -rf deps/skadi
git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/valhalla/skadi.git deps/skadi
pushd deps/skadi
scripts/dependencies.sh
scripts/install.sh
popd
