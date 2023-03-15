#!/bin/bash

# Update package list
sudo apt-get update

# Install PCL
sudo apt-get install libpcl-dev

# Clone and compile Eigen 3.3.9
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar xzf eigen-3.3.9.tar.gz
cd eigen-3.3.9
mkdir build && cd build
cmake ..
make
sudo make install

# Install LEMON
sudo apt-get install liblemon-dev

# Install OpenCV
sudo apt-get install libopencv-dev

# Install GEOS
sudo apt-get install libgeos-dev

# Install Boost
sudo apt-get install libboost-all-dev
