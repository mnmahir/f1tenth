#!/bin/bash
LIST_GIT_REPO=(     # List of git repositories to be cloned ("Git URL" "Commit ID/Tag/Branch"). Preferably use commit id / tag for reproducibility.
    "https://github.com/ros-navigation/navigation2.git ab8fa8215ff30033e9d213f7ca9cc1dc7216b670"
    "https://github.com/SteveMacenski/slam_toolbox.git 19a66904c648a174220be1a1b98e81a9ef0e6758"
    "https://github.com/cra-ros-pkg/robot_localization.git 51f48237ab8b60d27518dad3301ea8bd7138d155"
)


LIST_APT_PKG=(  # List of apt packages to be installed ("Package Name" "--optional-flag")
)


LIST_PYTHON_PKG=(   # List of python packages to be installed ("Package Name" "--optional-flag")
)


LIST_EXEC_CMD=(     # List of commands to be executed ("Command")
    "rm -rf $WS_PROJECT_REPO/pkg/_others/src/navigation2/nav2_system_tests"
)