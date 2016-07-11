#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo add-apt-repository -y ppa:kevinkreiser/prime-server
sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/kevinkreiser-prime-server-$(lsb_release -c -s).list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
sudo apt-get install -y autoconf automake pkg-config libtool make gcc g++ lcov vim-common jq libboost1.54-all-dev protobuf-compiler libprotobuf-dev libprime-server-dev

#clone async
mkdir -p deps
for dep in midgard baldr; do
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep &
done
wait

#build sync
for dep in midgard baldr; do
	pushd deps/$dep
	./autogen.sh
	./configure CPPFLAGS="-DBOOST_SPIRIT_THREADSAFE -DBOOST_NO_CXX11_SCOPED_ENUMS"
	make -j$(nproc)
	sudo make install
	popd
done
