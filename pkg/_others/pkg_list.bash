#!/bin/bash
LIST_GIT_REPO=(     # List of git repositories to be cloned ("Git URL" "Commit ID/Tag/Branch"). Preferably use commit id / tag for reproducibility.
    # Installed from apt below instead (official Jazzy releases of these repos). To patch one, uncomment it to build from source.
    # "https://github.com/ros-navigation/navigation2.git 645abd95f2be02a13ca539b29c2ddc065db34f89"    # jazzy branch
    # "https://github.com/SteveMacenski/slam_toolbox.git 02afdde003313a10b8d21461a92d9e5f4f0bc5f2"    # jazzy branch
    # "https://github.com/cra-ros-pkg/robot_localization.git 3efa714fb9c1ff40966327b7bed7053b2570be4d"    # jazzy-devel branch
)


LIST_APT_PKG=(  # List of apt packages to be installed ("Package Name" "--optional-flag")
    "ros-$ROS_DISTRO-navigation2"
    "ros-$ROS_DISTRO-slam-toolbox"
    "ros-$ROS_DISTRO-robot-localization"
    "ros-$ROS_DISTRO-rmw-zenoh-cpp"
)


LIST_PYTHON_PKG=(   # List of python packages to be installed ("Package Name" "--optional-flag")
)


LIST_EXEC_CMD=(     # List of commands to be executed ("Command")
)
