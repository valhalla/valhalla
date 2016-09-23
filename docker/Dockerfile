FROM ubuntu:14.04

# Install add-apt-repository and git
RUN apt-get update -y && apt-get install -y software-properties-common git

# Add Mapzen routing PPA and install Valhalla code
RUN add-apt-repository -y ppa:mapzen/routing && \
    apt-get update -y && \
    apt-get install -y valhalla-server

# Add Valhalla config
RUN mkdir -p /source && \
    cd /source && \
    git clone --depth=1 --branch=master --recurse-submodules https://github.com/valhalla/conf.git
