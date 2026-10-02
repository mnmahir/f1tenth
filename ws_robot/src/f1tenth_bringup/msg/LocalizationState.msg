# Is the car where the localization thinks it is? Published by f1tenth_tools/localization_monitor
std_msgs/Header header
bool active                 # a map is loaded
bool localized              # the lidar scan lines up with the map's walls at the estimated pose
float32 match               # 0..1, share of lidar points that land on the map's walls
float32 std_xy              # m, the localization's position uncertainty (1 sigma)
float32 std_yaw             # rad
float32 x                   # m, estimated pose of the car in the map
float32 y
float32 yaw                 # rad
string message
