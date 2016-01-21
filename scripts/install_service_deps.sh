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


# graph and install rapidjson
rm -rf rapidjson
git clone --depth=1 --recurse-submodules https://github.com/miloyip/rapidjson.git
pushd rapidjson
git submodule update --init
mkdir build && cd build
cmake ..
make
sudo make install
popd
