#!/bin/bash
set -e

# grab the latest zmq library:
git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/zeromq/libzmq.git
pushd libzmq
./autogen.sh && ./configure --without-libsodium --without-documentation && make -j4
sudo make install
popd
rm -rf libzmq

# grab experimental zmq-based server API:
git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/kevinkreiser/prime_server.git
pushd prime_server
./autogen.sh && ./configure && make test -j4
sudo make install
popd
rm -rf prime_server
