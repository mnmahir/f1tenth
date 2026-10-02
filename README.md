# F1Tenth

ROS 2 Jazzy stack (Ubuntu 24.04, Jetson Orin NX) for the F1TENTH car. Middleware: `rmw_zenoh_cpp`.

Everything is driven from one app, the **F1TENTH Pit Wall** UI. It runs on the car's screen or on a laptop. Users
never start nodes or launch files: the car's software starts at boot and waits, and the UI tells it what to run.

## Using the car
1. Switch the car on. Its software starts by itself (the `f1tenth` service) and waits on **standby**.
2. Open **F1TENTH Pit Wall** from the applications menu (or run `scripts/pit_wall.bash`). On a laptop, pick the car
   from the list; on the car it connects to itself.
3. On **HOME**, choose a session and press **START SESSION**:

| Session | What runs | Use it to |
|---|---|---|
| **Manual drive** | Sensors, VESC, joystick, safety. With a map: localization and lap timing. With a map and a path: path follower | Drive with the joystick. With a path: steering assist, or **autonomous** (button or joystick A, stop with B) |
| **Mapping** | Core + SLAM | Build a map, pause it, save it |
| **Path & raceline** | Core + localization on a map + path recorder + raceline optimizer | Record a lap and optimize it into a raceline, then drive it |
| **Standby** | Nothing | Stop everything |

Pages: **DRIVE** (3D view with bird / chase / onboard / orbit cameras, camera feed, speed, pedals, steering, battery,
timing, driver aids; it becomes **MAPPING** or **PATH** in those sessions), **GARAGE** (maps and paths on the car),
**TELEMETRY** (charts, G-force, laps), **SYSTEM** (components, nodes, sensor rates, car health, auto-restart,
rosbag recording, reboot / power off), **PARAMS** (any node's parameters, live, with **SAVE TO CAR**), **LOG** (ROS
log and the terminal output of the car's launch files).

### Driving autonomously
1. On **HOME**, start **Manual drive** with a map and a path (a recorded lap or an optimized raceline).
2. Put the car on the track, ideally on the path's start line, facing the way the path goes.
3. On **DRIVE**, under **Autonomous**, get **Localized** green: **START LINE** if the car is on the start line,
   **FIND CAR** to let it search the map for where its lidar scan fits, or **SET POSE** to click where it is and
   drag towards where it faces. When it's right, the red lidar points sit on the map's white walls.
4. When the checklist is green, press **ENGAGE AUTONOMOUS** (or **A** on the joystick). **Autonomous speed** sets
   the pace.
5. To stop: **DISENGAGE**, **B** or **RT** on the joystick, the **RB** kill switch, or **E-STOP**. The car also stops
   driving itself if it loses track of where it is for 1.5 s, and it never engages without a good localization.

The same localization buttons are on **PATH**: a recorded lap is only as good as the car's pose on the map.

**E-STOP** (top right, or the Space key) stops the car whatever is driving it, until released in the UI. On the
joystick: hold **LB** to drive (deadman), left stick throttle, right stick steering, **RT** brake, **LT** boost,
**RB** kill switch (LB releases it), **A** / **B** engage / disengage autonomous.

Power the car off from **SYSTEM** (power off car) before unplugging the battery.

## Car setup (once)
```bash
scripts/pkg_setup.bash            # clone + build everything in pkg/_*/pkg_list.bash, then ws_robot
scripts/interface_bringup.bash    # udev rules (/dev/sensors/*), dialout/input groups, Hokuyo Ethernet
scripts/install_car_service.bash  # run the car software at boot; name the car and give it its own ROS domain
scripts/install_ui_shortcut.bash  # "F1TENTH Pit Wall" in the applications menu
```
`scripts/pkg_setup.bash ws_robot` rebuilds only `ws_robot`; `NO_FETCH=1` skips apt/git/rosdep.
`install_car_service.bash --name <name> --domain <1-100>` renames the car or changes its domain;
`--remove` uninstalls the service. Service logs: `journalctl -u f1tenth -f`.

The Hokuyo UST-10LX is at `192.168.0.10` (see `ugv_bringup/config/sensor/lidar_hokuyo_ust10lx.yaml`);
`interface_bringup.bash` gives the Ethernet port `192.168.0.15/24` (NetworkManager connection `lidar-ethernet`).

## The UI on a laptop
With Ubuntu 24.04, ROS 2 Jazzy desktop and `ros-jazzy-rmw-zenoh-cpp`:
```bash
git clone -b jazzy https://github.com/mnmahir/f1tenth.git ~/f1tenth
cd ~/f1tenth/ws_robot && colcon build --packages-select f1tenth_bringup ugv_description
bash ~/f1tenth/scripts/install_ui_shortcut.bash   # or run ~/f1tenth/scripts/pit_wall.bash
```
Build the same version as the car: the UI and the car must share the same messages. `ugv_description` gives the 3D
car model.

**Finding the car.** Each car announces itself on the local network (UDP 47821) with its name, battery and session,
and the UI lists them. On another network (e.g. a VPN such as Tailscale), type the car's address in the list: the UI
asks the car directly (UDP 47820) and connects through the Zenoh router the car runs (TCP 7447). `pit_wall.bash
--car <name or address>` connects without asking; `--pick` always shows the list.

**Several cars.** Every car runs in its own ROS domain (set by `install_car_service.bash`; this car is `f1` on
domain 43). The UI joins only the domain of the car you pick, so it can't see or drive any other car. Give cars on
one network different names and domains.

## Development
The supervisor (`f1tenth_tools/supervisor`, started by `f1tenth_bringup car_launch.py`) runs each part of a
session as a launch file in `f1tenth_bringup/launch/components` and restarts parts that crash. The car-side tools
the UI needs are in `f1tenth_tools`: telemetry, drive coordinator (assist/autonomous), path recorder, raceline
optimizer, lap timer, MJPEG camera driver, system monitor. Maps, paths, saved parameters and bags are in
`~/f1tenth/data`; the components' output is in `~/.ros/f1tenth/*.log`.

The original launch files are still there for development. Stop the car service first (`sudo systemctl stop
f1tenth`), so the two don't fight over the hardware:
```bash
ros2 launch ugv_bringup bringup_launch.py   # robot, sensors, safety, EKF + AMCL localization
ros2 launch ugv_bringup mapping_launch.py   # robot, sensors, safety, EKF, slam_toolbox, avoidance controller
ros2 launch ugv_race race_launch.py         # pure pursuit + avoidance controller (with bringup)
ros2 launch ugv_race test_launch.py         # avoidance controller only, e.g. joystick driving (with bringup)
```
These load the map `ws_robot/src/ugv_race/maps/f1tenth/map.yaml` (override with `map:=<path>`).

Terminals on the car use the car's domain (`~/.config/f1tenth/car.env`, sourced from `~/.bashrc`).

### RViz on another computer
With the repository cloned as above, build the RViz configs and the car model, then use the car's domain and Zenoh
settings:
```bash
cd ~/f1tenth/ws_robot && colcon build --packages-select ugv_description ugv_analysis
source /opt/ros/jazzy/setup.bash && source ~/f1tenth/ws_robot/install/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_ROUTER_CHECK_ATTEMPTS=-1 ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:0"];scouting/multicast/enabled=true'
ros2 launch ugv_analysis analysis_launch.py   # car view; slam_launch.py while mapping
```
If that computer is also on other multicast networks (e.g. ZeroTier), limit discovery to the car's network by adding
`;scouting/multicast/interface="<interface>"` (e.g. `enp129s0`) to `ZENOH_CONFIG_OVERRIDE`. Otherwise it also finds
Zenoh peers there (log spam about `127.0.0.1` locators).

---
Pit wall UI and car software by Mahir Sehmi.
