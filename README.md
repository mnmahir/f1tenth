# F1Tenth

ROS 2 Jazzy stack (Ubuntu 24.04, Raspberry Pi 5) for the F1TENTH car. Middleware: `rmw_zenoh_cpp`.

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
Any number of laptops can run Pit Wall and connect to the car at the same time.

### Install
A laptop needs Ubuntu 24.04 and ROS 2 Jazzy Desktop
([install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)). The repository is private:
ask the owner to add your GitHub account (Settings → Collaborators), then:
```bash
sudo apt install -y ros-jazzy-desktop ros-jazzy-rmw-zenoh-cpp ros-dev-tools gh
gh auth login                       # sign in to GitHub
gh repo clone mnmahir/f1tenth ~/f1tenth
sudo rosdep init                    # once per computer; ignore "already initialized"
rosdep update
cd ~/f1tenth/ws_robot
rosdep install -y --ignore-src --rosdistro jazzy --from-paths src/f1tenth_bringup src/ugv_description
source /opt/ros/jazzy/setup.bash
colcon build --packages-select f1tenth_bringup ugv_description --cmake-args -DCMAKE_BUILD_TYPE=Release
bash ~/f1tenth/scripts/install_ui_shortcut.bash
```
Then open **F1TENTH Pit Wall** from the applications menu (or run `~/f1tenth/scripts/pit_wall.bash`). Without Qt and
RViz's development files the build still succeeds but leaves the UI out: `rosdep install` gets them.
`ugv_description` gives the 3D car model.

The UI and the car must share the same messages, so keep the laptop on the car's version. After the car is updated:
```bash
cd ~/f1tenth && git pull
cd ws_robot && source /opt/ros/jazzy/setup.bash
colcon build --packages-select f1tenth_bringup ugv_description --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Connecting to the car
- **Same network as the car** (Wi-Fi or a phone hotspot): the car announces itself every second (UDP 47821) and
  shows up in Pit Wall's list. Pick it and press **CONNECT**. There's no password: anyone on the network with Pit
  Wall can connect.
- **Car not in the list**: some networks block those announcements. Type the car's address and press **FIND** (the
  UI asks the car directly, UDP 47820). A connected Pit Wall shows the address when you click the car's name in the
  top bar.
- **Nothing gets through**: guest, campus and café Wi-Fi often stop devices from talking to each other. Use a phone
  hotspot that both the car and the laptops join, or Tailscale.
- **From anywhere, with Tailscale**: the car's owner shares it from the Tailscale admin console (Machines → the car
  → Share). Install Tailscale, accept the share, and type the car's Tailscale address (100.x.y.z) in the list.

Either way the UI connects as a Zenoh client of the router the car runs (TCP 7447), so the laptop only has to reach
that port on the car, whatever other networks or firewall it has. `pit_wall.bash --car <name or address>` connects
without asking; `--pick` always shows the list.

The car joins only Wi-Fi networks saved on it. To add one (on the car, or over SSH while it's on a network it knows):
`sudo nmcli device wifi connect "<network>" password "<password>"`.

### Several people
Everyone connected sees the same car and has full control: sessions, **E-STOP**, parameters, deleting maps and paths
(they go to a trash folder on the car), powering off. Agree who's in charge. The joystick is paired with the car, so
whoever holds it drives and has the **RB** kill switch. **ENGAGE AUTONOMOUS** also works from a Pit Wall when no
joystick is connected, and then only **E-STOP** stops the car: keep someone at the car whenever a session with a path
runs.

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
With the repository cloned as above, build the RViz configs and the car model, then join the car's domain as a client
of its Zenoh router (`<car>`: its address, e.g. `f1.barracuda-barley.ts.net` over Tailscale or its IP on the local
network):
```bash
cd ~/f1tenth/ws_robot && colcon build --packages-select ugv_description ugv_analysis
source /opt/ros/jazzy/setup.bash && source ~/f1tenth/ws_robot/install/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_ROUTER_CHECK_ATTEMPTS=-1 ZENOH_CONFIG_OVERRIDE='mode="client";scouting/multicast/enabled=false;connect/endpoints=["tcp/<car>:7447"]'
ros2 launch ugv_analysis analysis_launch.py   # car view; slam_launch.py while mapping
```
Avoid joining as a Zenoh peer (multicast discovery) from a computer that is also on other networks: discovery uses
one interface, and the car can't connect back to addresses it can't reach, so no data arrives.

---
Pit wall UI and car software by Mahir Sehmi.
