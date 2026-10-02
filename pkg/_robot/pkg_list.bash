#!/bin/bash
LIST_GIT_REPO=(     # List of git repositories to be cloned ("Git URL" "Commit ID/Tag/Branch"). Preferably use commit id / tag for reproducibility.
    "https://github.com/mnmahir/vesc.git f1tenth"  #forked from ros2 branch into f1tenth branch (adds imu_frame param, reversed odom)
    "https://github.com/ros-drivers/transport_drivers.git 0af598879560099b1069f9ad03d99f10684279ea" # main branch. Newer than the apt 1.2.0 release (serial port close/error fixes).

    # YDLIDAR TEA & SDK
    # "https://github.com/YDLIDAR/YDLidar-SDK.git a50614fac8dee55282fe8b7dbc105d88c3d5ee11"   # master branch
    # "https://github.com/YDLIDAR/ydlidar_ros2_driver.git 91e86db0d0197ffa6bd28f817b2b09756beff0a6" # humble branch

    # LDS01 LiDAR
    # "https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git ebf3ff75bbaa26edde1a24b855867e5e32d09c00"   # humble-devel branch

    # HOKUYO LiDAR (has the urg_library submodule)
    "https://github.com/Hokuyo-aut/urg_node2.git f02d8af22db2a4322217f4c5803494f4ca2903ff"    # main branch

)


LIST_APT_PKG=(  # List of apt packages to be installed ("Package Name" "--optional-flag")
)


LIST_PYTHON_PKG=(   # List of python packages to be installed ("Package Name" "--optional-flag")
)


LIST_EXEC_CMD=(     # List of commands to be executed ("Command")
    # # YDLIDAR SDK installation
    # "mkdir $WS_PROJECT_REPO/pkg/_robot/src/YDLidar-SDK/build"
    # "cd $WS_PROJECT_REPO/pkg/_robot/src/YDLidar-SDK/build"
    # "pwd"
    # "cmake .."
    # "make"
    # "sudo make install"
    # "cd -"
    # "pwd"

)
