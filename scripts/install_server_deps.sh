#!/bin/bash

# grab the latest zmq library:
git clone https://github.com/zeromq/libzmq.git
pushd libzmq
./autogen.sh && ./configure --without-libsodium && make check
sudo make install
popd
rm -rf libzmq

# grab experimental zmq-based server API:
git clone https://github.com/kevinkreiser/prime_server.git
pushd prime_server
./autogen.sh && ./configure --without-libsodium && make check
sudo make install
popd
rm -rf prime_server
