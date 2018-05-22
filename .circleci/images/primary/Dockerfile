FROM ubuntu:16.04

RUN apt-get update -y && apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:valhalla-core/valhalla && apt-get update -y
RUN apt-get install -y autoconf automake make libtool pkg-config g++ gcc jq lcov locales coreutils protobuf-compiler vim-common libboost-all-dev libcurl4-openssl-dev libgeos-dev libgeos++-dev liblua5.2-dev libprime-server0.6.3-dev libprotobuf-dev libspatialite-dev libsqlite3-dev libsqlite3-mod-spatialite python-all-dev zlib1g-dev liblz4-dev lua5.2 prime-server0.6.3-bin ccache curl clang-tidy-5.0 clang-5.0 git

# install cmake
ADD https://cmake.org/files/v3.11/cmake-3.11.2-Linux-x86_64.sh /tmp/cmake.sh
RUN sh /tmp/cmake.sh --prefix=/usr/local --skip-license && /bin/rm /tmp/cmake.sh
RUN cmake --version

ENV PATH /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV LD_LIBRARY_PATH /usr/local/lib:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/lib32:/usr/lib32
