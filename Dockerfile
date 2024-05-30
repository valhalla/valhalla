ARG BUILDER_IMAGE=ubuntu:23.04
ARG TARGET_IMAGE=ubuntu:23.04
ARG VALHALLA_VERSION=3.4.0

####################################################################
FROM $BUILDER_IMAGE as builder

ARG CONCURRENCY

USER root

# set paths
ENV LD_LIBRARY_PATH=/usr/local/lib:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/lib32:/usr/lib32

# install deps
RUN set -ex; \
  export DEBIAN_FRONTEND=noninteractive; \
  apt-get -qq update; \
  apt-get -y --no-install-recommends install \
  autoconf \
  automake \
  ccache \
  clang \
  clang-tidy \
  coreutils \
  curl \
  cmake \
  g++ \
  gcc \
  git \
  jq \
  lcov \
  libboost-all-dev \
  libcurl4-openssl-dev \
  libczmq-dev \
  libgdal-dev \
  libgeos++-dev \
  libgeos-dev \
  libluajit-5.1-dev \
  liblz4-dev \
  libprotobuf-dev \
  libspatialite-dev \
  libsqlite3-dev \
  libsqlite3-mod-spatialite \
  libtool \
  libzmq3-dev \
  lld \
  locales \
  luajit \
  make \
  osmium-tool \
  parallel \
  pkgconf \
  protobuf-compiler \
  python3-all-dev \
  python3-shapely \
  python3-pip \
  spatialite-bin \
  unzip \
  zlib1g-dev; \
  PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install --upgrade requests shapely; \
  rm -rf /var/lib/apt/lists/*

WORKDIR /usr/local/src/valhalla

ADD . .

WORKDIR /usr/local/src/valhalla/prime_server

RUN ./autogen.sh && ./configure
RUN make -j${CONCURRENCY:-$(nproc)}
RUN make install

# configure the build with symbols turned on so that crashes can be triaged
WORKDIR /usr/local/src/valhalla/build

# switch back to -DCMAKE_BUILD_TYPE=RelWithDebInfo and uncomment the block below if you want debug symbols
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc -DENABLE_SINGLE_FILES_WERROR=Off
RUN make all -j${CONCURRENCY:-$(nproc)}
RUN make install

# we wont leave the source around but we'll drop the commit hash we'll also keep the locales
WORKDIR /usr/local/src

RUN for f in valhalla/locales/*.json; do cat ${f} | python3 -c 'import sys; import json; print(json.load(sys.stdin)["posix_locale"])'; done > valhalla_locales
RUN rm -rf valhalla

####################################################################
FROM $TARGET_IMAGE as runner

USER root

# basic paths
ENV LD_LIBRARY_PATH=/usr/local/lib:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/lib32:/usr/lib32

# grab the builder stages artifacts
COPY --from=builder /usr/local /usr/local
COPY --from=builder /usr/lib/python3/dist-packages/valhalla/* /usr/lib/python3/dist-packages/valhalla/

# we need to add back some runtime dependencies for binaries and scripts
# install all the posix locales that we support
RUN export DEBIAN_FRONTEND=noninteractive && apt-get update -y && \
  apt-get install -y \
  libcurl4 libczmq4 libluajit-5.1-2 libgdal32 \
  libprotobuf-lite32 libsqlite3-0 libsqlite3-mod-spatialite libzmq5 zlib1g \
  curl gdb locales parallel python3-minimal python3-distutils python-is-python3 \
  spatialite-bin unzip wget && rm -rf /var/lib/apt/lists/*
RUN cat /usr/local/src/valhalla_locales | xargs -d '\n' -n1 locale-gen
RUN mkdir /data

# python smoke test
RUN python3 -c "import valhalla,sys; print(sys.version, valhalla)"

VOLUME /data

EXPOSE 8002
