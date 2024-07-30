# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.163.1/containers/cpp/.devcontainer/base.Dockerfile

# [Choice] Debian / Ubuntu version: debian-10, debian-9, ubuntu-20.04, ubuntu-18.04
ARG VARIANT="ubuntu-22.04"
FROM mcr.microsoft.com/vscode/devcontainers/cpp:0-${VARIANT}

RUN echo "Installing basic dependencies" && \
    apt-get update && \
    apt-get -y --no-install-recommends install \
    software-properties-common \
    build-essential \
    ca-certificates \
    libboost-all-dev \
    wget \
    libbz2-dev \
    libxml2-dev \
    libwebsocketpp-dev \
    autotools-dev \
    openssl \
    libssl-dev \
    libfmt-dev \
    ninja-build \
    libzip-dev \
    libsnappy-dev \
    lua5.2 \
    liblua5.2-dev \
    libtbb-dev \
    uuid-dev \
    default-jdk \
    protobuf-compiler-grpc \
    libgrpc++-dev \
    libgrpc-dev \
    libprotoc23 \
    libprotoc-dev \
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
    libcurl4-openssl-dev \
    libczmq-dev \
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
    zlib1g-dev \
    libpq-dev \
    -o APT::Install-Suggests=0 -o APT::Install-Recommends=0 && \
    update-ca-certificates



RUN echo "Install Valhalla dependencies" && \
    git clone --recurse-submodules https://github.com/kevinkreiser/prime_server /primeserver && \
    cd /primeserver && ./autogen.sh && ./configure && \
    make install && rm -r /primeserver && ldconfig

# for boost and scripts deps
RUN if [[ $(python3 -c 'import sys; print(int(sys.base_prefix != sys.prefix or hasattr(sys, "real_prefix")))') -eq 1 ]]; then \
        python3 -m pip install --upgrade requests shapely; \
    else \
        PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install --upgrade requests shapely; \
    fi

RUN pip install --upgrade "conan<2.0.0"

RUN echo "Building CMake from source" && \
    cmake --version && \
    rm -rf ~/cmake  && \
    mkdir ~/cmake && \
    cd ~/cmake && \
    wget https://cmake.org/files/v3.29/cmake-3.29.0.tar.gz && \
    tar -xzf cmake-3.29.0.tar.gz >/dev/null && \
    cd cmake-3.29.0/ && \
    ./bootstrap >/dev/null && \
    make -j$(nproc) && \
    sudo make install

RUN echo "Building Azure C Shared Utility from source" && \
    git clone --depth 1 --branch LTS_07_2022_Ref02 --recursive https://github.com/Azure/azure-c-shared-utility.git && \
    cd azure-c-shared-utility && \
    mkdir cmake && \
    cd cmake && \
    cmake .. && \
    cmake --build . && \
    make install

RUN echo "Building Azure uAMQP from source" && \
    git clone --depth 1 --branch LTS_07_2020_Ref02 --recursive https://github.com/Azure/azure-uamqp-c.git && \
    cd azure-uamqp-c && \
    cd deps/azure-c-shared-utility && git checkout LTS_07_2022_Ref02 && cd - && \
    mkdir cmake && \
    cd cmake && \
    cmake .. && \
    cmake --build . && \
    make install
