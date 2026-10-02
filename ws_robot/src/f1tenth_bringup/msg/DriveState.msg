# Who is driving, published by f1tenth_tools/drive_coordinator
std_msgs/Header header
uint8 IDLE=0
uint8 MANUAL=1
uint8 ASSIST=2              # joystick speed, path follower steering
uint8 AUTONOMOUS=3
uint8 source                # what produced the latest command
bool follower_available     # path follower output is fresh
bool assist_enabled
bool autonomous_enabled
string message
