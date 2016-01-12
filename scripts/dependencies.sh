#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo apt-get update
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/ubuntu-toolchain-r-test-$(lsb_release -c -s).list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
sudo apt-get install -y autoconf automake libtool make cmake gcc-4.9 g++-4.9 libboost1.54-dev libboost-program-options1.54-dev libboost-filesystem1.54-dev libboost-system1.54-dev libboost-thread1.54-dev lcov protobuf-compiler libprotobuf-dev libcurl4-openssl-dev
update-alternatives --remove-all gcc || true
update-alternatives --remove-all g++ || true
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 90

#clone async
mkdir -p deps
for dep in midgard baldr sif odin; do
	rm -rf $dep
	git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/valhalla/$dep.git deps/$dep &
done
wait

#install the service deps in the background
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
$DIR/install_service_deps.sh &
$DIR/install_yml_cpp.sh &

#build sync
for dep in midgard baldr sif; do
	pushd deps/$dep
	./autogen.sh
	./configure CPPFLAGS=-DBOOST_SPIRIT_THREADSAFE
	make -j4
	sudo make install
	popd
done
wait

#build sync
for dep in odin; do
        pushd deps/$dep
        ./autogen.sh
        ./configure CPPFLAGS=-DBOOST_SPIRIT_THREADSAFE
        make -j4
        sudo make install
        popd
done
