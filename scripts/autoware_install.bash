#!/bin/bash
PKG_WS_PATH=$WS_PROJECT_REPO/pkg/_others
AUTOWARE_PATH=$PKG_WS_PATH/src/autoware

# Dependencies
sudo apt install python3.10-venv
cd $AUTOWARE_PATH/

./setup-dev-env.sh --no-nvidia --no-cuda-drivers

#The NVIDIA library and cuda driver installation are disabled as they are already installed with the JetPack. If you force the cuda driver installation here, it can mess up the kernel and cause errors at bootup. You will need to reflash the JetPack if this happens.


mkdir $AUTOWARE_PATH/src
vcs import $AUTOWARE_PATH/src < $AUTOWARE_PATH/autoware.repos

source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update --include-eol-distros
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r
cd -

# #Ignore the `Invalid version` errors during rosdep installation
cd $PKG_WS_PATH
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cd -

