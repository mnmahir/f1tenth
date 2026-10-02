#!/bin/bash

# Author: Mahir Sehmi
# Description: Started by the f1tenth service (see install_car_service.bash): runs the car software with this
#              car's name, ROS domain and middleware settings.

REPO_PATH=$(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")
source "$REPO_PATH/setup.bash"
source "$HOME/.config/f1tenth/car.env"
exec ros2 launch f1tenth_bringup car_launch.py
