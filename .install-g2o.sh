#!/bin/sh

set -e

if [! -d "$G2O_INSTALL_DIR" ]; then
    cd g2o/
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX="$G2O_INSTALL_DIR" -DCMAKE_BUILD_TYPE=RELEASE ..
    make -j2
    make install
else
    echo "Using cached g2o install."
fi