#!/bin/bash
set -e

#grab specific release of yaml
wget https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.tar.gz -O yaml-cpp-release-0.5.2.tar.gz
tar pxvf yaml-cpp-release-0.5.2.tar.gz 
pushd yaml-cpp-release-0.5.2/
#concoct a makefile, build it, install it
cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON
make
sudo make install
popd
