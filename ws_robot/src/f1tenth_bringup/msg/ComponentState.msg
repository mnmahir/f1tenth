# One supervised launch component
string name
string status               # stopped | starting | running | stopping | failed
int32 pid
int32 restarts
float32 uptime              # s
string args
string[] missing_nodes      # expected nodes not in the ROS graph
string[] unresponsive_nodes # listed in the graph but not answering (hung)
