#!/bin/bash
NAME="SignalR-Client-Cpp"
VERSION="296b1228af824c68ab8973e4dc1249e78ae547e7"
REPO_LINK="https://github.com/StephanXu/SignalR-Client-Cpp.git"
source scripts/utilities.sh
sudo apt update -y
sudo apt install -y libcpprest-dev
pushd source
git clone "$REPO_LINK"
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