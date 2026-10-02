#!/bin/bash
LIST_GIT_REPO=(     # List of git repositories to be cloned ("Git URL" "Commit ID/Tag/Branch"). Preferably use commit id / tag for reproducibility.
)


LIST_APT_PKG=(  # List of apt packages to be installed ("Package Name" "--optional-flag")
    "ros-$ROS_DISTRO-rqt-common-plugins"
    "ros-$ROS_DISTRO-rviz2"
    "ros-$ROS_DISTRO-xacro"
    "ros-$ROS_DISTRO-joint-state-publisher-gui"
    "jstest-gtk"    # Joystick  GUI
    "python3-lark"
)


LIST_PYTHON_PKG=(   # List of python packages to be installed ("Package Name" "--optional-flag")
    # Ubuntu 24.04 blocks system-wide pip installs (PEP 668); prefer apt packages above.
)


LIST_EXEC_CMD=(     # List of commands to be executed ("Command")
    # Xpad driver to support F710 controller in JetPack 6 using X mode.
    # "sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4"
    # "sudo dkms install -m xpad -v 0.4"
)
