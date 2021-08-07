#!/bin/bash
source scripts/utilities.sh

sudo apt update -y
sudo apt install libssl1.0-dev -y
downloadFile https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz cmake-3.20.0.tar.gz

if [ ! -d "cmake-3.20.0" ]; then
    tar xvzf cmake-3.20.0.tar.gz
fi
pushd cmake-3.20.0
./configure --parallel=$NUM_JOBS --prefix=$INSTALL_PREFIX
make -j$NUM_JOBS
sudo make install
popd