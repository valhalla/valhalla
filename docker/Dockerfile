FROM ubuntu:24.04 AS builder
LABEL org.opencontainers.image.authors="Kevin Kreiser <kevinkreiser@gmail.com>"

# set paths
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/lib32:/usr/lib32
RUN export DEBIAN_FRONTEND=noninteractive && apt update && apt install -y sudo

# install deps
WORKDIR /src/valhalla
COPY scripts/install-linux-deps.sh scripts/install-linux-deps.sh
RUN bash ./scripts/install-linux-deps.sh

# get the code into the right place explicitly (for layer caching) and prepare to build it
COPY cmake cmake/
COPY date_time/ date_time/
COPY locales/ locales/
COPY lua/ lua/
COPY proto/ proto/
COPY scripts/ scripts/
COPY src/ src/
COPY third_party/ third_party/
COPY valhalla/ valhalla/
COPY CMakeLists.txt .
COPY libvalhalla.pc.in .
COPY LICENSE.md .
COPY CHANGELOG.md .
COPY COPYING .
COPY README.md .

ARG CONCURRENCY
ARG ADDITIONAL_TARGETS
# this arg will always compile the full project on every build bcs it's a commit hash
# but we only build the docker image in GHA if relevant files changed (extensive paths-ignore in GHA workflow)
ARG VERSION_MODIFIER=""
# switch back to -DCMAKE_BUILD_TYPE=RelWithDebInfo and uncomment the block below if you want debug symbols
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc -DENABLE_TESTS=Off -DENABLE_SINGLE_FILES_WERROR=Off -DVALHALLA_VERSION_MODIFIER=${VERSION_MODIFIER} && \
  make -C build all ${ADDITIONAL_TARGETS} -j${CONCURRENCY:-$(nproc)} && \
  make -C build install

# we wont leave the source around but we'll drop the commit hash and we'll also keep the locales
WORKDIR /usr/local/src
RUN echo "$VERSION_MODIFIER" > /usr/local/valhalla_version && \
  for f in /src/valhalla/locales/*.json; do cat ${f} | python3 -c 'import sys; import json; print(json.load(sys.stdin)["posix_locale"])'; done > valhalla_locales

# the binaries are huge with all the symbols so we strip them but keep the debug there if we need it
#WORKDIR /usr/local/bin
#RUN for f in valhalla_*; do objcopy --only-keep-debug $f $f.debug; done
#RUN tar -cvf valhalla.debug.tar valhalla_*.debug && gzip -9 valhalla.debug.tar
#RUN rm -f valhalla_*.debug
#RUN strip --strip-debug --strip-unneeded valhalla_* || true
#RUN strip /usr/local/lib/libvalhalla.a
#RUN strip /usr/local/lib/python3.12/dist-packages/valhalla/_valhalla*.so

###################################################################
# copy the important stuff from the build stage to the runner image
FROM ubuntu:24.04 AS runner
LABEL org.opencontainers.image.authors="Kevin Kreiser <kevinkreiser@gmail.com>"

# basic paths
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/lib32:/usr/lib32

# github packaging niceties
LABEL org.opencontainers.image.description="Open Source Routing Engine for OpenStreetMap and Other Datasources"
LABEL org.opencontainers.image.source="https://github.com/valhalla/valhalla"

# we need to add back some runtime dependencies for binaries and scripts
# install all the posix locales that we support
RUN export DEBIAN_FRONTEND=noninteractive && apt update && \
  apt install -y \
  libcurl4 libczmq4 libluajit-5.1-2 libgdal34 \
  libprotobuf-lite32 libsqlite3-0 libsqlite3-mod-spatialite libzmq5 zlib1g \
  curl locales parallel python3-minimal python-is-python3 python3-shapely python3-requests \
  spatialite-bin unzip wget

# grab the builder stages artifacts
COPY --from=builder /usr/local /usr/local
COPY --from=builder /usr/local/lib/python3.12/dist-packages/valhalla /usr/local/lib/python3.12/dist-packages/valhalla/

RUN cat /usr/local/src/valhalla_locales | xargs -d '\n' -n1 locale-gen

# python smoke test
RUN python3 -c "import valhalla, sys; print(sys.version, valhalla)"
