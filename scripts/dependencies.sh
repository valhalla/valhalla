#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo apt-get update
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/ubuntu-toolchain-r-test-$(lsb_release -c -s).list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
sudo apt-get install -y autoconf automake libtool make gcc-4.9 g++-4.9 libboost1.54-dev libboost-program-options1.54-dev libboost-filesystem1.54-dev libboost-system1.54-dev libboost-thread1.54-dev lcov protobuf-compiler libprotobuf-dev lua5.2 liblua5.2-dev libsqlite3-dev libspatialite-dev libgeos-dev libgeos++-dev
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 90

#skadi has everything
mkdir -p deps
rm -rf deps/skadi
git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/valhalla/skadi.git deps/skadi
pushd deps/skadi
scripts/dependencies.sh
scripts/install.sh
popd
