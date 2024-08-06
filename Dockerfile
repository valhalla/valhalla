# TODO: we should make use of BUILDPLATFORM and TARGETPLATFORM to figure out cross compiling
#  as mentioned here: docker.com/blog/faster-multi-platform-builds-dockerfile-cross-compilation-guide
#  then we could use the host architecture to simply compile to the target architecture without
#  emulating the target architecture (thereby making the build hyper slow). the general gist is
#  we add arm (or whatever architecture) repositories to apt and then install our dependencies
#  with the architecture suffix, eg. :arm64. then we just need to set a bunch of cmake variables
#  probably with the use of a cmake toolchain file so that cmake can make sure to use the
#  binaries that can target the target architecture. from there bob is your uncle maybe..

####################################################################
FROM ubuntu:24.04 as builder
MAINTAINER Kevin Kreiser <kevinkreiser@gmail.com>

ARG CONCURRENCY
ARG ADDITIONAL_TARGETS

# set paths
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/lib32:/usr/lib32
RUN export DEBIAN_FRONTEND=noninteractive && apt update && apt install -y sudo

# install deps
WORKDIR /usr/local/src/valhalla
COPY ./scripts/install-linux-deps.sh /usr/local/src/valhalla/scripts/install-linux-deps.sh
RUN bash /usr/local/src/valhalla/scripts/install-linux-deps.sh
RUN rm -rf /var/lib/apt/lists/*

# get the code into the right place and prepare to build it
ADD . .
RUN ls -la
RUN git submodule sync && git submodule update --init --recursive
RUN rm -rf build && mkdir build

# configure the build with symbols turned on so that crashes can be triaged
WORKDIR /usr/local/src/valhalla/build
# switch back to -DCMAKE_BUILD_TYPE=RelWithDebInfo and uncomment the block below if you want debug symbols
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc -DENABLE_SINGLE_FILES_WERROR=Off
RUN make all ${ADDITIONAL_TARGETS} -j${CONCURRENCY:-$(nproc)}
RUN make install

# we wont leave the source around but we'll drop the commit hash we'll also keep the locales
WORKDIR /usr/local/src
RUN cd valhalla && echo "https://github.com/valhalla/valhalla/tree/$(git rev-parse HEAD)" > ../valhalla_version
RUN for f in valhalla/locales/*.json; do cat ${f} | python3 -c 'import sys; import json; print(json.load(sys.stdin)["posix_locale"])'; done > valhalla_locales
RUN rm -rf valhalla

# the binaries are huge with all the symbols so we strip them but keep the debug there if we need it
#WORKDIR /usr/local/bin
#RUN for f in valhalla_*; do objcopy --only-keep-debug $f $f.debug; done
#RUN tar -cvf valhalla.debug.tar valhalla_*.debug && gzip -9 valhalla.debug.tar
#RUN rm -f valhalla_*.debug
#RUN strip --strip-debug --strip-unneeded valhalla_* || true
#RUN strip /usr/local/lib/libvalhalla.a
#RUN strip /usr/lib/python3/dist-packages/valhalla/python_valhalla*.so

####################################################################
# copy the important stuff from the build stage to the runner image
FROM ubuntu:24.04 as runner
MAINTAINER Kevin Kreiser <kevinkreiser@gmail.com>

# basic paths
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/lib32:/usr/lib32

# github packaging niceties
LABEL org.opencontainers.image.description = "Open Source Routing Engine for OpenStreetMap and Other Datasources"
LABEL org.opencontainers.image.source = "https://github.com/valhalla/valhalla"

# grab the builder stages artifacts
COPY --from=builder /usr/local /usr/local
COPY --from=builder /usr/local/lib/python3.12/dist-packages/valhalla/* /usr/local/lib/python3.12/dist-packages/valhalla/

# we need to add back some runtime dependencies for binaries and scripts
# install all the posix locales that we support
RUN export DEBIAN_FRONTEND=noninteractive && apt update && \
    apt install -y \
      libcurl4 libczmq4 libluajit-5.1-2 libgdal34 \
      libprotobuf-lite32 libsqlite3-0 libsqlite3-mod-spatialite libzmq5 zlib1g \
      curl gdb locales parallel python3-minimal python-is-python3 python3-shapely python3-requests \
      spatialite-bin unzip wget && rm -rf /var/lib/apt/lists/*
RUN cat /usr/local/src/valhalla_locales | xargs -d '\n' -n1 locale-gen

# python smoke test
RUN python3 -c "import valhalla,sys; print(sys.version, valhalla)"
