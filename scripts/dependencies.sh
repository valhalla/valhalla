#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ libboost1.54-all-dev
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

if [ -n "$1" ] && [ -d "$1" ]; then
        pushd "$1"
else
        pushd .
fi

#clone async
mkdir -p deps
for dep in midgard baldr; do
	rm -rf $dep
	git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/valhalla/$dep.git $dep &
done
wait

#install the service deps in the background
$DIR/install_service_deps.sh &

#build sync
for dep in midgard baldr; do
	pushd $dep
	./autogen.sh
	./configure CPPFLAGS=-DBOOST_SPIRIT_THREADSAFE
	make -j4
	sudo make install
	popd
done
wait

popd
