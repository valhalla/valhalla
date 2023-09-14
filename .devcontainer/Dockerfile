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
    pkg-config \
    protobuf-compiler \
    python3-all-dev \
    python3-shapely \
    python3-pip \
    spatialite-bin \
    unzip \
    zlib1g-dev \
    -o APT::Install-Suggests=0 -o APT::Install-Recommends=0 && \
    update-ca-certificates

RUN echo "Install Valhalla dependencies" && \
    git clone --recurse-submodules https://github.com/kevinkreiser/prime_server /primeserver && \
    cd /primeserver && ./autogen.sh && ./configure && \
    make install && rm -r /primeserver && ldconfig

RUN pip install --upgrade "conan<2.0.0"

RUN echo "Building oatpp from source" && \
    git clone https://github.com/oatpp/oatpp.git --branch 1.3.0 && \
    cd oatpp/ && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make install

RUN echo "Building prometheus from source" && \
    git clone --recurse-submodules https://github.com/jupp0r/prometheus-cpp.git --branch v1.1.0 && \
    cd prometheus-cpp/ && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_SHARED_LIBS=ON -DENABLE_PUSH=OFF -DENABLE_COMPRESSION=OFF && \
    cmake --build . --parallel 4 && \
    ctest -V && \
    cmake --install . 

