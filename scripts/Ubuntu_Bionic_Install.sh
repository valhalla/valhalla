#!/bin/bash

export NODEJS_INCLUDE_DIRS=$HOME/.nvm/versions/node/v10.15.0/include

sudo apt update
sudo apt upgrade -y

sudo apt install -y mc

sudo apt install -y cmake make libtool pkg-config g++ gcc jq lcov protobuf-compiler vim-common libboost-all-dev libboost-all-dev libcurl4-openssl-dev zlib1g-dev liblz4-dev libprotobuf-dev


#if you plan to compile with data building support, see below for more info

sudo apt install -y libgeos-dev libgeos++-dev liblua5.2-dev libspatialite-dev libsqlite3-dev lua5.2 wget
if [[ $(grep -cF bionic /etc/lsb-release) > 0 ]]; then sudo apt install -y libsqlite3-mod-spatialite; fi


#if you plan to compile with python bindings, see below for more info

sudo apt install -y python-all-dev


#if you plan to compile with node bindings, run
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash

export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion

nvm install 10
nvm use install 10 && nvm use 10 # must use node 8.11.1 and up because of N-API

ln -s /root/.nvm/versions/node/v10.15.0/include/node/node.h /usr/include/node.h
ln -s /root/.nvm/versions/node/v10.15.0/include/node/uv.h /usr/include/uv.h
ln -s /root/.nvm/versions/node/v10.15.0/include/node/v8.h /usr/include/v8.h


cd $HOME

git clone https://github.com/valhalla/valhalla.git

cd $HOME/valhalla

npm install --ignore-scripts

# Clone and build prime_server

cd $HOME

git clone https://github.com/kevinkreiser/prime_server.git


# grab some standard autotools stuff
sudo apt install -y autoconf automake libtool make gcc g++ lcov 
# grab curl (for url de/encode) and zmq for the awesomeness
sudo apt install -y libcurl4-openssl-dev libzmq3-dev libczmq-dev

cd $HOME/prime_server

# dont forget submodules
git submodule update --init --recursive
# standard autotools:
./autogen.sh
./configure
make test -j8
sudo make install

echo -e "\nexport LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> /etc/profile
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/

cd $HOME/valhalla

git submodule update --init --recursive
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

