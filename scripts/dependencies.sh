#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ lcov
=======
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ vim-common libboost1.54-all-dev lcov

#clone async
mkdir -p deps
for dep in midgard; do
=======
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ lcov libboost1.54-all-dev

#clone async
mkdir -p deps
for dep in midgard baldr; do
>>>>>>> sif/master
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep &
=======
=======
>>>>>>> skadi/master
if [[ $(grep -cF trusty /etc/lsb-release) > 0 ]]; then
  sudo add-apt-repository -y ppa:kevinkreiser/libsodium
  sudo add-apt-repository -y ppa:kevinkreiser/libpgm
  sudo add-apt-repository -y ppa:kevinkreiser/zeromq3
  sudo add-apt-repository -y ppa:kevinkreiser/czmq
fi
sudo add-apt-repository -y ppa:kevinkreiser/prime-server
sudo apt-get update
<<<<<<< HEAD
sudo apt-get install -y autoconf automake libtool make pkg-config gcc g++ lcov libboost-all-dev libprime-server0.6.3-dev

#clone async
mkdir -p deps
for dep in midgard baldr sif; do
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep & 
>>>>>>> meili/master
=======
sudo apt-get install -y autoconf automake libtool make pkg-config gcc g++ lcov libboost1.54-all-dev libprime-server0.6.3-dev

#clone async
mkdir -p deps
for dep in midgard baldr; do
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep & 
>>>>>>> skadi/master
done
wait

#build sync
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
for dep in midgard; do
=======
for dep in midgard baldr; do
>>>>>>> sif/master
=======
for dep in midgard baldr sif; do
>>>>>>> meili/master
=======
for dep in midgard baldr; do
>>>>>>> skadi/master
	pushd deps/$dep
	./autogen.sh
	./configure CPPFLAGS="-DBOOST_SPIRIT_THREADSAFE -DBOOST_NO_CXX11_SCOPED_ENUMS"
	make -j$(nproc)
	sudo make install
	popd
done
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> baldr/master
=======
>>>>>>> sif/master
=======
wait
>>>>>>> meili/master
=======
wait
>>>>>>> skadi/master
