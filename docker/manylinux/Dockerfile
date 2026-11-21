# - upstream: https://github.com/pypa/manylinux:manylinux_2_28_*
# - builds multiple packages from source which are too old or missing from dnf
# - source installs go to /opt/valhalla-deps so they don't shadow system packages (e.g. sqlite3)

ARG ARCH=x86_64
ARG MANYLINUX_TAG=2026.11.14-1

FROM quay.io/pypa/manylinux_2_28_${ARCH}:${MANYLINUX_TAG} AS base

ENV VALHALLA_DEPS=/opt/valhalla-deps
RUN dnf -y install \
      ccache ninja-build zip wget perl-IPC-Cmd \
      lz4-devel czmq-devel openssl-devel libcurl-devel luajit-devel \
      libtiff-devel libgeotiff-devel \
      proj-devel readosm-devel minizip-devel libxml2-devel \
    && dnf clean all

# prefer our source builds
ENV PATH=${VALHALLA_DEPS}/bin:${PATH} \
    CMAKE_PREFIX_PATH=${VALHALLA_DEPS} \
    PKG_CONFIG_PATH=${VALHALLA_DEPS}/lib/pkgconfig:${PKG_CONFIG_PATH} \
    LD_LIBRARY_PATH=${VALHALLA_DEPS}/lib:${LD_LIBRARY_PATH}

# build the 
FROM base AS builder

RUN dnf -y install autoconf automake libtool && dnf clean all
COPY fetch_release.sh /usr/local/bin/fetch_release
# autotools projects find our prefix through these, cmake ones through CMAKE_PREFIX_PATH
ENV CPPFLAGS="-I${VALHALLA_DEPS}/include" \
    LDFLAGS="-L${VALHALLA_DEPS}/lib" \
    CMAKE_BUILD_TYPE=Release \
    CMAKE_GENERATOR=Ninja \
    CMAKE_INSTALL_PREFIX=${VALHALLA_DEPS}
WORKDIR /tmp/build

# for spatialite we need sqlite compiled with rtree + column metadata support
RUN fetch_release https://www.sqlite.org/2026/sqlite-autoconf-3540100.tar.gz \
      926ef8d3eab93fbf6e06011901273eec076b25594a4e1cdadd50c987f4f6d64e \
    && cd sqlite-autoconf-3540100 \
    && ./configure --prefix=${VALHALLA_DEPS} --enable-all \
      CFLAGS="-O2 -DSQLITE_ENABLE_RTREE=1 -DSQLITE_ENABLE_COLUMN_METADATA=1" \
    && make -j$(nproc) && make install \
    && rm ${VALHALLA_DEPS}/lib/libsqlite3.a && rm -rf /tmp/build/*

# AlmaLinux 8 has 3.7.2 without a .pc file
RUN fetch_release https://github.com/libgeos/geos/releases/download/3.13.1/geos-3.13.1.tar.bz2 \
      df2c50503295f325e7c8d7b783aca8ba4773919cde984193850cf9e361dfd28c \
    && cmake -S geos-3.13.1 -B geos-build -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_TESTING=OFF \
    && cmake --build geos-build && cmake --install geos-build && rm -rf /tmp/build/*

# not packaged; spatialite needs it for ST_MakeValid in valhalla_build_admins
RUN fetch_release https://github.com/CGX-GROUP/librttopo/archive/refs/tags/librttopo-1.1.0.tar.gz \
      3b2b6a6e7b05b9e0d07afda13d5e26466677abfff736bfa75034e05d205226a4 \
    && cd librttopo-librttopo-1.1.0 && ./autogen.sh \
    && ./configure --prefix=${VALHALLA_DEPS} --disable-static \
    && make -j$(nproc) && make install && rm -rf /tmp/build/*

# the gaia-gis tarballs ship a config.guess that predates aarch64
RUN fetch_release https://www.gaia-gis.it/gaia-sins/freexl-2.0.0.tar.gz \
      176705f1de58ab7c1eebbf5c6de46ab76fcd8b856508dbd28f5648f7c6e1a7f0 \
    && cd freexl-2.0.0 && cp -f /usr/share/automake-*/config.{guess,sub} . \
    && ./configure --prefix=${VALHALLA_DEPS} --disable-static \
    && make -j$(nproc) && make install && rm -rf /tmp/build/*

RUN fetch_release https://www.gaia-gis.it/gaia-sins/libspatialite-sources/libspatialite-5.1.0.tar.gz \
      43be2dd349daffe016dd1400c5d11285828c22fea35ca5109f21f3ed50605080 \
    && cd libspatialite-5.1.0 && cp -f /usr/share/automake-*/config.{guess,sub} . \
    && ./configure --prefix=${VALHALLA_DEPS} --disable-static --enable-rttopo \
    && make -j$(nproc) && make install && rm -rf /tmp/build/*

RUN fetch_release https://www.gaia-gis.it/gaia-sins/spatialite-tools-sources/spatialite-tools-5.1.0.tar.gz \
      df3030367c089ca90fa6630897f3f1a280784da29e1ba634f340dba4b08583b5 \
    && cd spatialite-tools-5.1.0 && cp -f /usr/share/automake-*/config.{guess,sub} . \
    && ./configure --prefix=${VALHALLA_DEPS} \
    && make -j$(nproc) && make install && rm -rf /tmp/build/*

# AlmaLinux 8 has 1.66, we need >= 1.71; headers only
RUN fetch_release https://github.com/boostorg/boost/releases/download/boost-1.88.0/boost-1.88.0-b2-nodocs.tar.gz \
      85138e4a185a7e7535e82b011179c5b5fb72185bea9f59fe8e2d76939b2f5c51 \
    && mkdir -p ${VALHALLA_DEPS}/include && cp -r boost-1.88.0/boost ${VALHALLA_DEPS}/include/ \
    && rm -rf /tmp/build/*

# AlmaLinux 8 has 3.5.0
RUN fetch_release https://github.com/protocolbuffers/protobuf/releases/download/v21.12/protobuf-cpp-3.21.12.tar.gz \
      4eab9b524aa5913c6fffb20b2a8abf5ef7f95a80bc0701f3a6dbb4c607f73460 \
    && cmake -S protobuf-3.21.12 -B protobuf-build -DCMAKE_INSTALL_LIBDIR=lib \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON -Dprotobuf_BUILD_TESTS=OFF \
    && cmake --build protobuf-build && cmake --install protobuf-build && rm -rf /tmp/build/*

RUN git clone --depth 1 --branch 0.13.1 --recurse-submodules --shallow-submodules \
      https://github.com/kevinkreiser/prime_server \
    && cmake -S prime_server -B prime_server-build -DCMAKE_INSTALL_LIBDIR=lib \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DENABLE_WALL=OFF -DENABLE_WERROR=OFF \
    && cmake --build prime_server-build && cmake --install prime_server-build && rm -rf /tmp/build/*

# remove docs and strip the binaries; strip errors on non-ELF files like geos-config
RUN rm -rf ${VALHALLA_DEPS}/share/man ${VALHALLA_DEPS}/share/doc
RUN strip --strip-unneeded ${VALHALLA_DEPS}/bin/* ${VALHALLA_DEPS}/lib/*.so* 2>/dev/null || true

FROM base
COPY --from=builder ${VALHALLA_DEPS} ${VALHALLA_DEPS}
