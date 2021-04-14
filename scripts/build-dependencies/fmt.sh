#!/bin/bash
NAME="fmt"
VERSION="7.1.3"
REPO_LINK="https://github.com/fmtlib/fmt.git"
source scripts/utilities.sh
pushd source
git clone --branch "$VERSION" "$REPO_LINK"
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