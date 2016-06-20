#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ vim-common libboost1.54-all-dev lcov

#clone async
mkdir -p deps
for dep in midgard; do
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep &
done
wait

#build sync
for dep in midgard; do
	pushd deps/$dep
	./autogen.sh
	./configure CPPFLAGS="-DBOOST_SPIRIT_THREADSAFE -DBOOST_NO_CXX11_SCOPED_ENUMS"
	make -j$(nproc)
	sudo make install
	popd
done
