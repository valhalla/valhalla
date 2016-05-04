#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
./autogen.sh
./configure --enable-coverage CPPFLAGS=-DBOOST_SPIRIT_THREADSAFE
make CPPFLAGS=-DNDEBUG CXXFLAGS="-std=c++11 -O3" test -j$(nproc)
sudo make install
sudo ldconfig
