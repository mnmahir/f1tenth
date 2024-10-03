#!/bin/bash
LIST_GIT_REPO=(     # List of git repositories to be cloned ("Git URL" "Commit ID/Tag/Branch"). Preferably use commit id / tag for reproducibility.
    "https://github.com/f1tenth/f1tenth_system.git 2a780e00103ffa6d62300b5b86a543d6d0664a4e"    # humble-devel branch
    "https://github.com/f1tenth/ackermann_mux.git b3c0b083ac03aa8c648537d7e4d22608fcd3440c" #foxy-devel branch
    "https://github.com/f1tenth/teleop_tools.git 4337558fd38e0e1a5c08bdd3c7b4b1a9454a74a9"  #foxy-devel branch
    "https://github.com/f1tenth/f1tenth_gym.git cd56335eda43ff4e401331c461877227474a3ed4"   # main branch
    "https://github.com/f1tenth/f1tenth_gym_ros.git 151820202d6b530491d22c308f22638182f1298d"   # main branch
    "https://github.com/f1tenth/particle_filter.git ec599a1c4f3d4edc5f7d356e2f2990839e56bb7d"   # humble-devel branch
)


LIST_APT_PKG=(  # List of apt packages to be installed ("Package Name" "--optional-flag")
)


LIST_PYTHON_PKG=(   # List of python packages to be installed ("Package Name" "--optional-flag")
)


LIST_EXEC_CMD=(     # List of commands to be executed ("Command")
    "rm -rf $WS_PROJECT_REPO/pkg/_f1tenth/src/f1tenth_system/ackermann_mux $WS_PROJECT_REPO/pkg/_f1tenth/src/f1tenth_system/teleop_tools $WS_PROJECT_REPO/pkg/_f1tenth/src/f1tenth_system/vesc"
    "touch $WS_PROJECT_REPO/pkg/_f1tenth/src/f1tenth_gym/COLCON_IGNORE"
    "pip3 install -e $WS_PROJECT_REPO/pkg/_f1tenth/src/f1tenth_gym"
)