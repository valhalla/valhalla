#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`

#deps
#sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo rm -f -f /etc/apt/sources.list.d/neo4j.list
sudo apt-get update
sudo apt-get install -y autoconf automake libtool make gcc-4.8 g++-4.8 lcov
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 90

#test
./autogen.sh
./configure --enable-coverage
make test -j4
sudo make install
