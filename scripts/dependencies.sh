#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo apt-get install -y autoconf automake libtool make gcc-4.8 g++-4.8 libboost1.54-dev libboost-program-options1.54-dev libboost-filesystem1.54-dev libboost-system1.54-dev libboost-thread1.54-dev lcov libcurl4-openssl-dev geotiff-bin libgeotiff-dev libgeotiff2
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 90
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
	git clone --depth=1 --recurse-submodules --single-branch --branch=master https://github.com/valhalla/$dep.git deps/$dep &
done
wait

#get newer gdal
$DIR/install_gdal.sh "$1" &

#install the service deps in the background
$DIR/install_service_deps.sh &

#build sync
for dep in midgard baldr; do
	pushd deps/$dep
	./autogen.sh
	./configure CPPFLAGS=-DBOOST_SPIRIT_THREADSAFE
	make -j4
	sudo make install
	popd
done
wait

popd
