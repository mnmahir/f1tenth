#!/bin/bash
LIST_GIT_REPO=(     # List of git repositories to be cloned ("Git URL" "Commit ID/Tag/Branch"). Preferably use commit id / tag for reproducibility.
    "https://github.com/f1tenth/f1tenth_system.git cffbbd7a89d14d59bfd6123d8abdc5ad4d955105"    # jazzy-devel branch
    "https://github.com/f1tenth/ackermann_mux.git b3c0b083ac03aa8c648537d7e4d22608fcd3440c" # foxy-devel branch (what f1tenth_system jazzy-devel pins)
    "https://github.com/f1tenth/teleop_tools.git 163827aa96039a70f5fe7f2bb59c332417b64eee"  # humble-devel branch (what f1tenth_system jazzy-devel pins). Keeps joy_teleop's "default" action that upstream lacks.
    "https://github.com/f1tenth/particle_filter.git ec599a1c4f3d4edc5f7d356e2f2990839e56bb7d"   # humble-devel branch (latest ROS 2 branch). Runtime needs RangeLibc python wrappers, not installed here.

    # Simulator: not needed on the car. The Jazzy bridge needs f1tenth_gym dev-jax (NumPy 2, JAX) installed in a venv,
    # which would clash with ROS's system NumPy if pip-installed globally. See f1tenth_gym_ros README (dev-jazzy).
    # "https://github.com/f1tenth/f1tenth_gym_ros.git 2c019810b4098ed82d44cad1d7d282a2dfc12421"   # dev-jazzy branch
    # "https://github.com/f1tenth/f1tenth_gym.git ec3b1a4693ed20317e42e57a9454d94b824cec81"   # dev-jax branch
)


LIST_APT_PKG=(  # List of apt packages to be installed ("Package Name" "--optional-flag")
    "python3-transforms3d"  # particle_filter (instead of its pip rosdep key)
)


LIST_ROSDEP_SKIP_KEYS=(    # rosdep keys not to install
    "sick_scan_xd"              # f1tenth_stack: SICK LiDAR, not on this car
    "rosbridge_server"          # f1tenth_stack: web bridge, not used
    "vesc"                      # f1tenth_stack: our fork is built in _robot
    "python-transforms3d-pip"   # particle_filter: pip is blocked on Ubuntu 24.04, apt package above
)


LIST_PYTHON_PKG=(   # List of python packages to be installed ("Package Name" "--optional-flag")
)


LIST_EXEC_CMD=(     # List of commands to be executed ("Command")
    # f1tenth_system's submodules are cloned separately above (vesc is our fork in _robot)
    "rm -rf $WS_PROJECT_REPO/pkg/_f1tenth/src/f1tenth_system/ackermann_mux $WS_PROJECT_REPO/pkg/_f1tenth/src/f1tenth_system/teleop_tools $WS_PROJECT_REPO/pkg/_f1tenth/src/f1tenth_system/vesc"
)
