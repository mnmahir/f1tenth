#!/bin/bash
LIST_GIT_REPO=(     # List of git repositories to be cloned ("Git URL" "Commit ID/Tag/Branch"). Preferably use commit id / tag for reproducibility.
    "https://github.com/f1tenth/f1tenth_gym.git cd56335eda43ff4e401331c461877227474a3ed4"
    "https://github.com/f1tenth/f1tenth_gym_ros.git 910789ad9029839abda0d7b6d66f46945fe5cef0"
)


LIST_APT_PKG=(  # List of apt packages to be installed ("Package Name" "--optional-flag")
)


LIST_PYTHON_PKG=(   # List of python packages to be installed ("Package Name" "--optional-flag")
)


LIST_EXEC_CMD=(     # List of commands to be executed ("Command")
    "touch $WS_PROJECT_REPO/pkg/_sims/src/f1tenth_gym/COLCON_IGNORE"
    "pip3 install -e $WS_PROJECT_REPO/pkg/_sims/src/f1tenth_gym"
)