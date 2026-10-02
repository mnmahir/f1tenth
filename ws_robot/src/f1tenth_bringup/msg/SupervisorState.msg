# Session state, published by f1tenth_tools/supervisor
std_msgs/Header header
string mode                 # idle | manual | mapping | path
string map                  # selected map name, '' for none
string path                 # selected path name, '' for none
bool auto_restart
bool bag_recording
bool mapping_paused          # SLAM ignores new scans (mapping mode)
ComponentState[] components
string[] maps
string[] paths
string[] path_maps          # the map each path was made on ('' if unknown), same order as paths
string maps_dir
string paths_dir
string message              # latest event
