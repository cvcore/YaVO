#!/bin/sh

set -e -u

if [ ! -d "$G2O_INSTALL_DIR/lib" ]; then
    cd g2o/
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX="$G2O_INSTALL_DIR" -DCMAKE_BUILD_TYPE=RELEASE -DG2O_BUILD_EXAMPLES=OFF -DG2O_BUILD_APPS=OFF ..
    make -j2
    make install
else
    echo "Using cached g2o install."
fi
