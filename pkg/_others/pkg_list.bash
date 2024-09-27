#!/bin/bash
LIST_GIT_REPO=(     # List of git repositories to be cloned ("Git URL" "Commit ID/Tag/Branch"). Preferably use commit id / tag for reproducibility.
    "https://github.com/ros-navigation/navigation2.git f79de33c3c192cd66377dc9538bdcbc3597b7f66"    # humble branch
    "https://github.com/SteveMacenski/slam_toolbox.git 19a66904c648a174220be1a1b98e81a9ef0e6758"    # humble branch
    "https://github.com/cra-ros-pkg/robot_localization.git 663436c73675d72f8a6c6217e2e9ec7a605bb558"    #humble-devel branch
    "https://github.com/autowarefoundation/autoware.git 61224c9b9c1d6019023a60913e05d04d7d374337"   #f1tenth_humble branch
)


LIST_APT_PKG=(  # List of apt packages to be installed ("Package Name" "--optional-flag")
    "ros-$ROS_DISTRO-rmw-cyclonedds-cpp"
)


LIST_PYTHON_PKG=(   # List of python packages to be installed ("Package Name" "--optional-flag")
)


LIST_EXEC_CMD=(     # List of commands to be executed ("Command")
    "rm -rf $WS_PROJECT_REPO/pkg/_others/src/navigation2/nav2_system_tests"
)

LIST_ECHO_ENDING_MSG=(  # List of ending messages to be displayed at the end ("Message")
     "echo -e '$BASH_WARNING Autoware need to be installed manually. Use $WS_PROJECT_REPO/scripts/autoware_install.bash'"
)