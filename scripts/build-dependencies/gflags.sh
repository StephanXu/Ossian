#!/bin/bash
NAME="gflags"
VERSION="v2.2.2"
GITHUB_LINK="https://github.com/gflags/gflags.git"
source scripts/utilities.sh
pushd source
git clone --branch "$VERSION" "$GITHUB_LINK"
pushd $NAME
mkdir build.release
pushd build.release
doConfigure $NAME "cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
      ../"
doMake $NAME
doInstall $NAME
popd
popd