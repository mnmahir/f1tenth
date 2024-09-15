#!/bin/bash
LIST_GIT_REPO=(     # List of git repositories to be cloned ("Git URL" "Commit ID/Tag/Branch"). Preferably use commit id / tag for reproducibility.
)


LIST_APT_PKG=(  # List of apt packages to be installed ("Package Name" "--optional-flag")
    "ros-$ROS_DISTRO-rqt*"
    "ros-$ROS_DISTRO-rviz2"
    "ros-$ROS_DISTRO-rviz*"
    "jstest-gtk"    # Joystick  GUI
)


LIST_PYTHON_PKG=(   # List of python packages to be installed ("Package Name" "--optional-flag")
    "lark"
    "xacro"
)


LIST_EXEC_CMD=(     # List of commands to be executed ("Command")
    # Xpad driver to support F710 controller in JetPack 6 using X mode.
    # "sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4"    
    # "sudo dkms install -m xpad -v 0.4"
)