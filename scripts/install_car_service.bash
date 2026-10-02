#!/bin/bash

# Author: Mahir Sehmi
# Description: Installs the car service: at boot it runs the F1TENTH car software (the supervisor the UI talks to,
#              f1tenth_bringup car_launch.py). Nothing else runs until a session is started from the UI.
#              Also gives the car a name and a ROS domain of its own, so UIs list cars by name and a laptop can
#              never drive the wrong car. Terminals on the car get the same settings through ~/.bashrc.
#              Run it on the car. Safe to re-run (e.g. to rename the car).
#              Usage: bash scripts/install_car_service.bash [--name NAME] [--domain ID]
#                     bash scripts/install_car_service.bash --remove

set -e
CURRENT_SCRIPT_PATH=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
REPO_PATH=$(dirname "$CURRENT_SCRIPT_PATH")

BASH_INFO=${BASH_INFO:-"\e[34m[INFO]\e[0m"}
BASH_SUCCESS=${BASH_SUCCESS:-"\e[32m[SUCCESS]\e[0m"}
BASH_ERROR=${BASH_ERROR:-"\e[31m[ERROR]\e[0m"}

SERVICE=f1tenth
UNIT=/etc/systemd/system/$SERVICE.service
ENV_FILE=$HOME/.config/f1tenth/car.env
BLOCK_BEGIN="# >>> f1tenth car >>>"
BLOCK_END="# <<< f1tenth car <<<"

remove_bashrc_block() {
    sed -i "/^$BLOCK_BEGIN\$/,/^$BLOCK_END\$/d" "$HOME/.bashrc"
}

if [ "$1" = "--remove" ]; then
    sudo systemctl disable --now $SERVICE 2>/dev/null || true
    sudo rm -f $UNIT
    sudo systemctl daemon-reload
    remove_bashrc_block
    rm -f "$ENV_FILE"
    echo -e "$BASH_SUCCESS Car service removed. Start the car software by hand with: ros2 launch f1tenth_bringup car_launch.py"
    exit 0
fi

NAME=$(hostname)
DOMAIN=${ROS_DOMAIN_ID:-}
while [ $# -gt 0 ]; do
    case $1 in
        --name) NAME=$2; shift 2 ;;
        --domain) DOMAIN=$2; shift 2 ;;
        *) echo "Usage: $0 [--name NAME] [--domain ID] | --remove"; exit 1 ;;
    esac
done
if ! [[ "$NAME" =~ ^[A-Za-z0-9_-]{1,32}$ ]]; then
    echo -e "$BASH_ERROR Car name '$NAME': use up to 32 letters, digits, - or _"
    exit 1
fi
if [ -z "$DOMAIN" ] || [ "$DOMAIN" = "0" ]; then
    # Its own domain, derived from the name. Cars on one network must differ: pick with --domain if two collide.
    DOMAIN=$(( $(printf '%s' "$NAME" | cksum | cut -d' ' -f1) % 100 + 1 ))
fi
if ! [[ "$DOMAIN" =~ ^[0-9]+$ ]] || [ "$DOMAIN" -lt 1 ] || [ "$DOMAIN" -gt 100 ]; then
    echo -e "$BASH_ERROR ROS domain '$DOMAIN': use 1 to 100"
    exit 1
fi

# The car software's environment: Zenoh (routerless, peers find each other by multicast) plus name and domain.
# Middleware settings already in this shell are kept.
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}
ZENOH_ROUTER_CHECK_ATTEMPTS=${ZENOH_ROUTER_CHECK_ATTEMPTS:--1}
ZENOH_CONFIG_OVERRIDE=${ZENOH_CONFIG_OVERRIDE:-'listen/endpoints=["tcp/0.0.0.0:0"];scouting/multicast/enabled=true'}
mkdir -p "$(dirname "$ENV_FILE")"
{
    echo "# The F1TENTH car's identity and ROS middleware settings (written by scripts/install_car_service.bash)."
    echo "# Used by the car service, the UI shortcut and terminals on the car."
    for VAR in RMW_IMPLEMENTATION ZENOH_ROUTER_CHECK_ATTEMPTS ZENOH_CONFIG_OVERRIDE ZENOH_SESSION_CONFIG_URI; do
        [ -n "${!VAR}" ] && printf 'export %s=%q\n' "$VAR" "${!VAR}"
    done
    printf 'export ROS_DOMAIN_ID=%q\n' "$DOMAIN"
    printf 'export F1TENTH_CAR_NAME=%q\n' "$NAME"
} > "$ENV_FILE"
echo -e "$BASH_INFO Car '$NAME' on ROS domain $DOMAIN (settings in $ENV_FILE)"

# Terminals on the car use the same domain, so ros2 CLI tools see the car's nodes
remove_bashrc_block
{
    echo "$BLOCK_BEGIN"
    echo "# This car's name and ROS domain (scripts/install_car_service.bash). Remove with --remove."
    echo "[ -f \"$ENV_FILE\" ] && source \"$ENV_FILE\""
    echo "$BLOCK_END"
} >> "$HOME/.bashrc"

chmod +x "$CURRENT_SCRIPT_PATH/car_service.bash" "$CURRENT_SCRIPT_PATH/pit_wall.bash"
sudo tee $UNIT > /dev/null <<UNIT_EOF
[Unit]
Description=F1TENTH car software: the supervisor the pit wall UI talks to
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=$USER
ExecStart=$CURRENT_SCRIPT_PATH/car_service.bash
# SIGINT to ros2 launch only: it stops the supervisor, which stops the session's nodes
KillMode=mixed
KillSignal=SIGINT
TimeoutStopSec=45
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
UNIT_EOF
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE > /dev/null 2>&1
sudo systemctl restart $SERVICE
sleep 2
if systemctl is-active --quiet $SERVICE; then
    echo -e "$BASH_SUCCESS The car service is running and starts at every boot."
else
    echo -e "$BASH_ERROR The car service didn't start: journalctl -u $SERVICE -e"
    exit 1
fi
echo -e "$BASH_INFO Logs: journalctl -u $SERVICE -f   ·   Stop: sudo systemctl stop $SERVICE   ·   Remove: $0 --remove"
echo -e "$BASH_INFO Open a new terminal (or: source ~/.bashrc) so ros2 tools use domain $DOMAIN."
