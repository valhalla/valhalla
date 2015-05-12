#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo rm -f -f /etc/apt/sources.list.d/neo4j.list
sudo apt-get install -y autoconf automake libtool make gcc-4.8 g++-4.8 libboost1.54-dev libboost-system1.54-dev libboost-thread1.54-dev lcov
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 90

#clone async
mkdir -p deps
for dep in midgard; do
	git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/valhalla/$dep.git deps/$dep &
done
wait

#build sync
for dep in midgard; do
	pushd $dep
	./autogen.sh
	./configure CPPFLAGS=-DBOOST_SPIRIT_THREADSAFE
	make -j4
	sudo make install
	popd
done
