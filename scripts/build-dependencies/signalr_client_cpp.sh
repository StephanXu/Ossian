#!/bin/bash
NAME="SignalR-Client-Cpp"
VERSION="0b86c8ec6aa02f4dd3e92c172f9b0a516f06e728"
GITHUB_LINK="https://github.com/StephanXu/SignalR-Client-Cpp.git"
source scripts/utilities.sh
sudo apt update -y
sudo apt install -y libcpprest-dev
pushd source
git clone "$GITHUB_LINK"
pushd $NAME
git checkout -b "$VERSION"
git submodule update --init
mkdir build.release
pushd build.release
doConfigure $NAME "cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
      -DUSE_CPPRESTSDK=true \
      -DCMAKE_PREFIX_PATH=/usr/lib/aarch64-linux-gnu/cmake/ \
      ../"
doMake $NAME
doInstall $NAME
popd
popd