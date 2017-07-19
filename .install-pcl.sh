#!/bin/sh

set -e -u

if [ ! -d "$PCL_INSTALL_DIR/lib" ]; then
    wget https://github.com/PointCloudLibrary/pcl/archive/pcl-$PCL_VERSION.tar.gz
    tar xzf pcl-$PCL_VERSION.tar.gz
    rm pcl-$PCL_VERSION.tar.gz

    cd pcl-pcl-$PCL_VERSION
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PCL_INSTALL_DIR ..
    make -j4
    make install
else
    echo "Using cached PCL install."
fi

