#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> mjolnir/master
=======
>>>>>>> odin/master
=======
>>>>>>> loki/master
=======
>>>>>>> thor/master
=======
>>>>>>> tyr/master
=======
>>>>>>> tools/master
if [[ $(grep -cF trusty /etc/lsb-release) > 0 ]]; then
  sudo add-apt-repository -y ppa:kevinkreiser/libsodium
  sudo add-apt-repository -y ppa:kevinkreiser/libpgm
  sudo add-apt-repository -y ppa:kevinkreiser/zeromq3
  sudo add-apt-repository -y ppa:kevinkreiser/czmq
fi
sudo add-apt-repository -y ppa:kevinkreiser/prime-server
sudo apt-get update
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
sudo apt-get install -y autoconf automake libtool make pkg-config gcc g++ lcov libboost-all-dev libprime-server0.6.3-dev
=======
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ lcov libboost1.54-all-dev libprime-server0.6.3-dev
>>>>>>> loki/master

#clone async
mkdir -p deps
for dep in midgard baldr sif; do
<<<<<<< HEAD
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep & 
>>>>>>> meili/master
=======
sudo apt-get install -y autoconf automake libtool make pkg-config gcc g++ lcov libboost1.54-all-dev libprime-server0.6.3-dev
=======
sudo apt-get install -y autoconf automake pkg-config libtool make gcc g++ lcov vim-common jq libboost1.54-all-dev protobuf-compiler libprotobuf-dev libprime-server0.6.3-dev
>>>>>>> odin/master

#clone async
mkdir -p deps
for dep in midgard baldr; do
<<<<<<< HEAD
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep & 
>>>>>>> skadi/master
=======
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ lcov vim-common libboost1.54-all-dev protobuf-compiler libprotobuf-dev lua5.2 liblua5.2-dev libsqlite3-dev libspatialite-dev libgeos-dev libgeos++-dev libcurl4-openssl-dev libprime-server0.6.3-dev

#clone async
mkdir -p deps
for dep in midgard baldr skadi; do
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep &
>>>>>>> mjolnir/master
=======
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep &
>>>>>>> odin/master
=======
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep &
>>>>>>> loki/master
=======
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config cmake pkg-config gcc g++ lcov libboost1.54-all-dev protobuf-compiler libprotobuf-dev libprime-server0.6.3-dev

#clone async
mkdir -p deps
for dep in midgard baldr sif meili odin; do
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep &
>>>>>>> thor/master
=======
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ lcov libboost1.54-all-dev protobuf-compiler libprotobuf-dev libprime-server0.6.3-dev

#clone async
mkdir -p deps
for dep in midgard baldr odin; do
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep &
>>>>>>> tyr/master
=======
sudo apt-get install -y autoconf automake pkg-config libtool make pkg-config gcc g++ lcov libboost1.54-all-dev libprotobuf-dev libprime-server0.6.3-dev prime-server0.6.3-bin vim-common protobuf-compiler lua5.2 liblua5.2-dev libsqlite3-dev libspatialite-dev libgeos-dev libgeos++-dev libcurl4-openssl-dev

#clone async
mkdir -p deps
for dep in midgard baldr skadi mjolnir sif meili loki odin thor tyr; do
	git clone --depth=1 --recurse --single-branch https://github.com/valhalla/$dep.git deps/$dep &
>>>>>>> tools/master
done
wait

#build sync
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
for dep in midgard baldr skadi; do
>>>>>>> mjolnir/master
=======
for dep in midgard baldr; do
>>>>>>> odin/master
=======
for dep in midgard baldr sif; do
>>>>>>> loki/master
=======
for dep in midgard baldr sif meili odin; do
>>>>>>> thor/master
=======
for dep in midgard baldr odin; do
>>>>>>> tyr/master
=======
for dep in midgard baldr sif meili skadi mjolnir loki odin thor tyr; do
>>>>>>> tools/master
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> mjolnir/master
=======
>>>>>>> odin/master
=======
>>>>>>> loki/master
=======
>>>>>>> thor/master
=======
>>>>>>> tyr/master
=======
>>>>>>> tools/master
