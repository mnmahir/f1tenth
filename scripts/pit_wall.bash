#!/bin/bash

# Author: Mahir Sehmi
# Description: Starts the F1TENTH Pit Wall UI. Used by the menu shortcut (install_ui_shortcut.bash) and fine to run
#              from a terminal. The UI finds the cars on the network and sets up the ROS middleware itself; on the
#              car it uses the car's own settings.

REPO_PATH=$(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")
source "$REPO_PATH/setup.bash"
[ -f "$HOME/.config/f1tenth/car.env" ] && source "$HOME/.config/f1tenth/car.env"
exec ros2 run f1tenth_bringup f1tenth_ui "$@"
