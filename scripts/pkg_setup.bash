#!/bin/bash
# Fetch, install and build everything listed in pkg/_*/pkg_list.bash, then build ws_robot.
# Stand-in for the old dev manager's package step; pkg_list.bash files keep the same format.
#
# Usage:
#   scripts/pkg_setup.bash                 # every pkg/_* group, then ws_robot
#   scripts/pkg_setup.bash _robot ws_robot # only the named groups/workspaces, in that order
#   NO_FETCH=1 scripts/pkg_setup.bash      # skip apt/pip/git/rosdep, just rebuild
set -eo pipefail

export WS_PROJECT_REPO="$(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

INFO="\e[34m[INFO]\e[0m"
WARN="\e[33m[WARN]\e[0m"
ERROR="\e[31m[ERROR]\e[0m"

# ROS setup files are not written for errexit/nounset
source_ws() {
	set +eu
	source "$1"
	set -e
}

ensure_rosdep() {
	if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
		sudo rosdep init
	fi
	if [ ! -d "$HOME/.ros/rosdep/sources.cache" ] || [ -n "$(find "$HOME/.ros/rosdep/sources.cache" -maxdepth 0 -mtime +7)" ]; then
		rosdep update --rosdistro "$ROS_DISTRO"
	fi
}

# clone_repo <url> <ref> <dest_dir>: clone or update a repo and check out <ref> (commit, tag or branch).
clone_repo() {
	local url=$1 ref=$2 dir=$3
	if [ ! -d "$dir/.git" ]; then
		echo -e "$INFO Cloning $url"
		git clone -q "$url" "$dir"
	elif [ -n "$(git -C "$dir" status --porcelain --ignore-submodules)" ]; then
		echo -e "$WARN $dir has local changes, leaving it at $(git -C "$dir" rev-parse --short HEAD)"
		return
	else
		git -C "$dir" fetch -q --tags origin
	fi
	# A branch name means "latest of that branch"
	local target=$ref
	if git -C "$dir" rev-parse -q --verify "origin/$ref^{commit}" >/dev/null; then
		target="origin/$ref"
	fi
	git -C "$dir" -c advice.detachedHead=false checkout -q --detach "$target"
	git -C "$dir" submodule update -q --init --recursive
	echo -e "$INFO $(basename "$dir") @ $ref ($(git -C "$dir" rev-parse --short HEAD))"
}

setup_group() {
	local ws=$1
	local list="$ws/pkg_list.bash"
	local LIST_GIT_REPO=() LIST_APT_PKG=() LIST_PYTHON_PKG=() LIST_EXEC_CMD=() LIST_ROSDEP_SKIP_KEYS=()
	[ -f "$list" ] && source "$list"

	if [ -z "$NO_FETCH" ]; then
		if [ ${#LIST_APT_PKG[@]} -gt 0 ]; then
			echo -e "$INFO apt: ${LIST_APT_PKG[*]}"
			sudo apt-get install -y ${LIST_APT_PKG[@]}
		fi
		for pkg in "${LIST_PYTHON_PKG[@]}"; do
			python3 -m pip install $pkg
		done
		mkdir -p "$ws/src"
		for entry in "${LIST_GIT_REPO[@]}"; do
			read -r url ref <<<"$entry"
			clone_repo "$url" "$ref" "$ws/src/$(basename "$url" .git)"
		done
		for cmd in "${LIST_EXEC_CMD[@]}"; do
			echo -e "$INFO exec: $cmd"
			eval "$cmd"
		done
	fi

	if [ -z "$(find "$ws/src" -name package.xml -print -quit 2>/dev/null)" ]; then
		echo -e "$INFO $(basename "$ws"): nothing to build"
		return
	fi
	if [ -z "$NO_FETCH" ]; then
		ensure_rosdep
		rosdep install -q -y -r --from-paths "$ws/src" --ignore-src --rosdistro "$ROS_DISTRO" \
			${LIST_ROSDEP_SKIP_KEYS:+--skip-keys "${LIST_ROSDEP_SKIP_KEYS[*]}"}
	fi
	echo -e "$INFO Building $(basename "$ws")"
	(cd "$ws" && colcon build $COLCON_ARGS --cmake-args -DCMAKE_BUILD_TYPE=Release)
}

if [ $# -eq 0 ]; then
	set -- $(cd "$WS_PROJECT_REPO/pkg" && ls -d _*/ | tr -d /) ws_robot
fi

source_ws "/opt/ros/$ROS_DISTRO/setup.bash"
# Overlay groups already built so later ones can depend on them
for ws in "$WS_PROJECT_REPO"/pkg/_*/; do
	[ -f "$ws/install/local_setup.bash" ] && source_ws "$ws/install/local_setup.bash"
done

for name in "$@"; do
	case $name in
		ws_*) ws="$WS_PROJECT_REPO/$name" ;;
		*) ws="$WS_PROJECT_REPO/pkg/$name" ;;
	esac
	if [ ! -d "$ws" ]; then
		echo -e "$ERROR No such group or workspace: $name"
		exit 1
	fi
	echo -e "$INFO ===== $name ====="
	if [[ $name == ws_* ]]; then
		COLCON_ARGS="--symlink-install" setup_group "$ws"
	else
		setup_group "$ws"
	fi
	[ -f "$ws/install/local_setup.bash" ] && source_ws "$ws/install/local_setup.bash"
done

echo -e "$INFO Done. Open a new terminal or run: source $WS_PROJECT_REPO/setup.bash"
