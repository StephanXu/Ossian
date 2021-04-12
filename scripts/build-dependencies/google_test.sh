#!/bin/bash
NAME="googletest"
VERSION="release-1.10.0"
GITHUB_LINK="https://github.com/google/googletest.git"
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