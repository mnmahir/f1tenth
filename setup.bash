# Source ROS 2 and every built workspace of this repo: pkg/_* overlays first, then ws_robot.
# Build them with scripts/pkg_setup.bash.
_f1tenth_repo="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
for _f1tenth_ws in "$_f1tenth_repo"/pkg/_*/ "$_f1tenth_repo/ws_robot/"; do
	[ -f "$_f1tenth_ws/install/local_setup.bash" ] && source "$_f1tenth_ws/install/local_setup.bash"
done
unset _f1tenth_repo _f1tenth_ws
