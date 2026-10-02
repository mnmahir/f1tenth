# Lap timing on the selected path, published by f1tenth_tools/lap_timer
std_msgs/Header header
bool active                 # a path is loaded and the car is localized
uint32 lap                  # completed laps
float32 current_lap_time    # s
float32 last_lap_time       # s, 0 if none
float32 best_lap_time       # s, 0 if none
float32 progress            # 0..1 along the path
float32 cross_track_error   # m, distance from the path
