FROM gyllen/mapillary_base_ubuntu:0.0.3

RUN mkdir -p /source

RUN apt-get update -y

RUN cd /source && git clone --depth=1 --branch=master --recurse-submodules http://github.com/valhalla/meili.git && \
    cd meili && \
    ./scripts/dependencies.sh && \
    ./scripts/install.sh && \
    rm -rf /source/meili

# Dependencies of Mjolnir
RUN apt-get install -y protobuf-compiler libprotobuf-dev lua5.2 liblua5.2-dev libsqlite3-dev libspatialite-dev libgeos-dev libgeos++-dev libcurl4-openssl-dev

# Skadi and Mjolnir are optional. Comment them out if you get them
# installed on your host machine
RUN cd /source && git clone --depth=1 --recurse-submodules https://github.com/valhalla/skadi.git && \
    cd skadi && \
    ./scripts/install.sh && \
    rm -rf /source/skadi

RUN cd /source && git clone --depth=1 --recurse-submodules https://github.com/valhalla/mjolnir.git && \
    cd mjolnir && \
    ./scripts/install.sh && \
    rm -rf /source/mjolnir

RUN ldconfig
