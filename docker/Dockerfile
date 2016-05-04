FROM gyllen/mapillary_base_ubuntu:0.0.3


RUN add-apt-repository -y ppa:kevinkreiser/prime-server && \
    apt-get update -o Dir::Etc::sourcelist="sources.list.d/kevinkreiser-prime-server-$(lsb_release -c -s).list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0" && \
    apt-get update -y && apt-get install -y \
    git wget \
    autoconf automake libtool pkg-config make cmake gcc g++ lcov \
    libboost1.54-all-dev \
    protobuf-compiler \
    libprotobuf-dev \
    libprime-server-dev

RUN mkdir /source
WORKDIR /source


RUN git clone --depth=1 --recurse-submodules https://github.com/valhalla/midgard.git && \
    cd midgard && \
    ./scripts/install.sh && \
    rm -rf /source/midgard


RUN git clone --depth=1 --recurse-submodules https://github.com/valhalla/baldr.git && \
    cd baldr && \
    ./scripts/install.sh \
    rm -rf /source/baldr


RUN git clone --depth=1 --recurse-submodules https://github.com/valhalla/sif.git && \
    cd sif && \
    ./scripts/install.sh && \
    rm -rf /source/sif


# Skadi and Mjolnir are optional
RUN apt-get install -y lua5.2 liblua5.2-dev libsqlite3-dev libspatialite-dev libgeos-dev libgeos++-dev libcurl4-openssl-dev
RUN git clone --depth=1 --recurse-submodules https://github.com/valhalla/skadi.git && \
    cd skadi && \
    ./scripts/install.sh && \
    rm -rf /source/skadi

RUN git clone --depth=1 --recurse-submodules https://github.com/valhalla/mjolnir.git && \
    cd mjolnir && \
    ./scripts/install.sh && \
    rm -rf /source/mjolnir


# Install stable RapidJSON
ENV RAPIDJSON_VERSION 1.0.2
RUN wget https://github.com/miloyip/rapidjson/archive/v${RAPIDJSON_VERSION}.tar.gz -O rapidjson.tar.gz && \
    tar xf rapidjson.tar.gz && \
    cd rapidjson-${RAPIDJSON_VERSION} && \
    mkdir -p build && cd build && \
    cmake .. && make && sudo make install && \
    rm -rf /source/rapidjson-${RAPID_VERSION} /source/rapidjson.tar.gz


# RUN git clone --depth=1 --recurse-submodules https://github.com/miloyip/rapidjson.git && \
#     cd rapidjson && \
#     git submodule update --init && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make && make install && \
#     rm -rf /source/rapidjson


RUN git clone --depth=1 --branch=development --recurse-submodules http://github.com/mapillary/map_matching_plus.git && \
    cd map_matching_plus && \
    ./scripts/install.sh && \
    rm -rf /source/map_matching_plus


RUN ldconfig
