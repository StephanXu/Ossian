#!/bin/bash
NAME="benchmark"
VERSION="v1.5.2"
GITHUB_LINK="https://github.com/google/benchmark.git"
source scripts/utilities.sh
pushd source
git clone --branch "$VERSION" "$GITHUB_LINK"
pushd $NAME
mkdir build.release
pushd build.release
doConfigure $NAME "cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
      -D BENCHMARK_DOWNLOAD_DEPENDENCIES=True \
      ../"
doMake $NAME
doInstall $NAME
popd
popd