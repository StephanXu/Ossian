#!/bin/bash
NAME="json"
VERSION="v3.9.1"
REPO_LINK="https://github.com/nlohmann/json.git"
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