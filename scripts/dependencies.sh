#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo add-apt-repository -y ppa:kevinkreiser/prime-server
sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/kevinkreiser-prime-server-$(lsb_release -c -s).list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
sudo apt-get install -y autoconf automake libtool make pkg-config cmake gcc g++ lcov libboost1.54-all-dev protobuf-compiler libprotobuf-dev libprime-server-dev

#clone async
mkdir -p deps
for dep in midgard baldr sif; do
    rm -rf $dep
    git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/valhalla/$dep.git deps/$dep &
done
wait

#build sync
for dep in midgard baldr sif; do
    pushd deps/$dep
    ./scripts/install.sh
    popd
done
wait

# Grab and install RapidJSON
RAPIDJSON_VERSION=1.0.2
rm -rf rapidjson.tar.gz
wget https://github.com/miloyip/rapidjson/archive/v${RAPIDJSON_VERSION}.tar.gz -O rapidjson.tar.gz
## Or you can:
# git clone --depth=1 --recurse-submodules https://github.com/miloyip/rapidjson.git
rm -rf rapidjson-${RAPIDJSON_VERSION}
tar xf rapidjson.tar.gz

pushd rapidjson-${RAPIDJSON_VERSION}
## Need if you grab by git clone
# git submodule update --init
mkdir build && cd build
cmake ..
make
sudo make install
popd
