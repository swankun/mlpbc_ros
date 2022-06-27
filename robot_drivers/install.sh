#!/usr/bin/env bash

# Make working directory in /tmp
ROOTDIR=/tmp/lely
mkdir -p $ROOTDIR
cd $ROOTDIR

# Install dependencies for LelyCAN
sudo apt install -y \
    wget build-essential automake libtool \
    python3-setuptools python3-wheel \
    python3-empy python3-yaml \
    libbluetooth-dev \
    valgrind \
    doxygen graphviz

# Download Lely release and extract it 
LELY_VERSION=v2.3.1
wget https://gitlab.com/lely_industries/lely-core/-/archive/$LELY_VERSION/lely-core-$LELY_VERSION.tar.gz -O - | tar -xz
cd lely-core-$LELY_VERSION

# Build
autoreconf -i
mkdir build && cd build
../configure --disable-cython
make -j$(nproc)
make install 

