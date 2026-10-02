#!/bin/bash
#
# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA Corporation and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA Corporation is strictly prohibited.
#

# 4.14.0 is the first release that builds with CUDA 13.2 (JetPack 7 / L4T r39)
version="4.14.0"
folder="workspace"
py_ver=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')

set -e

for (( ; ; ))
do
    echo "Do you want to remove the default OpenCV (yes/no)? Note: this also removes ROS packages built against it (e.g. cv_bridge)."
    read rm_old

    if [ "$rm_old" = "yes" ]; then
        echo "** Remove other OpenCV first"
        sudo apt -y purge *libopencv*
	break
    elif [ "$rm_old" = "no" ]; then
	break
    fi
done


echo "------------------------------------"
echo "** Install requirement (1/4)"
echo "------------------------------------"
sudo apt-get update
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python3-dev python3-numpy
sudo apt-get install -y libtbb-dev libjpeg-dev libpng-dev libtiff-dev libv4l-dev v4l-utils qv4l2
sudo apt-get install -y curl unzip


echo "------------------------------------"
echo "** Download opencv "${version}" (2/4)"
echo "------------------------------------"
mkdir -p $folder
cd ${folder}
echo "*" > .gitignore
curl -L https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
curl -L https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
unzip -q -o opencv-${version}.zip
unzip -q -o opencv_contrib-${version}.zip
rm opencv-${version}.zip opencv_contrib-${version}.zip
cd opencv-${version}/


echo "------------------------------------"
echo "** Build opencv "${version}" (3/4)"
echo "------------------------------------"
mkdir -p release
cd release/
# Python bindings go to /usr/local/lib/python3.X/dist-packages, which is already on Ubuntu's sys.path
cmake -D WITH_CUDA=ON -D WITH_CUDNN=ON -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda -D CUDA_ARCH_BIN="8.7" -D CUDA_ARCH_PTX="" -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_PYTHON3_INSTALL_PATH=/usr/local/lib/python${py_ver}/dist-packages -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${version}/modules -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python3=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)


echo "------------------------------------"
echo "** Install opencv "${version}" (4/4)"
echo "------------------------------------"
sudo make install
sudo ldconfig    # /usr/local/lib is already in the loader path, no LD_LIBRARY_PATH needed


echo "** Install opencv "${version}" successfully"
echo "** Bye :)"
