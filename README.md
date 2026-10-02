# F1Tenth

ROS 2 Jazzy stack (Ubuntu 24.04, Jetson Orin NX) for the F1TENTH car. Middleware: `rmw_zenoh_cpp`.

## Setup
```bash
scripts/pkg_setup.bash          # clone + build everything in pkg/_*/pkg_list.bash, then ws_robot
scripts/interface_bringup.bash  # udev rules (/dev/sensors/*), dialout/input groups, Hokuyo Ethernet
source setup.bash               # sources ROS, pkg/_* and ws_robot (already in ~/.bashrc on the car)
```
`scripts/pkg_setup.bash ws_robot` rebuilds only `ws_robot`; `NO_FETCH=1` skips apt/git/rosdep.

The Hokuyo UST-10LX is at `192.168.0.10` (see `ugv_bringup/config/sensor/lidar_hokuyo_ust10lx.yaml`);
`interface_bringup.bash` gives the Ethernet port `192.168.0.15/24` (NetworkManager connection `lidar-ethernet`).

## Run
```bash
ros2 launch ugv_bringup bringup_launch.py   # robot, sensors, safety, EKF + AMCL localization
ros2 launch ugv_bringup mapping_launch.py   # robot, sensors, safety, EKF, slam_toolbox, avoidance controller
ros2 launch ugv_race race_launch.py         # pure pursuit + avoidance controller (with bringup)
ros2 launch ugv_race test_launch.py         # avoidance controller only, e.g. joystick driving (with bringup)
```
The VESC only gets commands through the avoidance controller (`/ackermann_cmd` -> `/ackermann_cmd_filtered`),
so run `race_launch.py` or `test_launch.py` next to `bringup_launch.py` to drive.

## Maps
Localization loads `ws_robot/src/ugv_race/maps/f1tenth/map.yaml` (override with `map:=<path>`); without it,
bringup skips map_server and amcl. To make one, run `mapping_launch.py`, drive the track, then:
```bash
mkdir -p ~/f1tenth/ws_robot/src/ugv_race/maps/f1tenth
ros2 run nav2_map_server map_saver_cli -f ~/f1tenth/ws_robot/src/ugv_race/maps/f1tenth/map
cd ~/f1tenth/ws_robot && colcon build --symlink-install   # installs the new map files
```
