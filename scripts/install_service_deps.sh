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
wget -O prime_server0.3.2.tar.gz https://github.com/kevinkreiser/prime_server/archive/0.3.2.tar.gz
tar pxvf prime_server0.3.2.tar.gz
pushd prime_server-0.3.2
./autogen.sh
./configure
make -j4
sudo make install
popd
