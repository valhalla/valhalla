#!/bin/bash
set -e

export LD_LIBRARY_PATH=.:`cat /etc/ld.so.conf.d/* | grep -v -E "#" | tr "\\n" ":" | sed -e "s/:$//g"`
sudo apt-get update
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/ubuntu-toolchain-r-test-$(lsb_release -c -s).list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
sudo apt-get install -y autoconf automake libtool make gcc-4.9 g++-4.9 lcov
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 90
