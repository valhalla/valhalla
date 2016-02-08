#!/bin/bash
set -e

# grab the latest zmq library:
rm -rf libzmq
git clone --recurse-submodules --single-branch --branch=master https://github.com/zeromq/libzmq.git
pushd libzmq
git checkout b3f2acf7d625daef65d37ffa00dfed753cf2387b
./autogen.sh
./configure --without-libsodium --without-documentation --without-vmci
make -j4
sudo make install
popd

# grab experimental zmq-based server API:
rm -rf prime_server
git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/kevinkreiser/prime_server.git
pushd prime_server
./autogen.sh
./configure
make -j4
sudo make install
popd


# grab and install rapidjson

## Or you can:
# rm -rf rapidjson
# git clone --depth=1 --recurse-submodules https://github.com/miloyip/rapidjson.git

RAPIDJSON_VERSION=1.0.2
rm -rf rapidjson.tar.gz
wget https://github.com/miloyip/rapidjson/archive/v${RAPIDJSON_VERSION}.tar.gz -O rapidjson.tar.gz
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
