#!/bin/bash

# Author: Mahir Sehmi
# Description: Sets up the robot's sensor interfaces.
#              Installs the udev rules in usb_rules/ (stable /dev/sensors/* and /dev/input/* names),
#              puts the user in the groups needed to open the devices, and gives the Ethernet port
#              a static IP on the Hokuyo LiDAR's subnet. Prints the status of each device.
#              Safe to re-run. Usage: bash scripts/interface_bringup.bash

# get current script path
CURRENT_SCRIPT_PATH=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
REPO_PATH=$(dirname "$CURRENT_SCRIPT_PATH")

BASH_INFO=${BASH_INFO:-"\e[34m[INFO]\e[0m"}
BASH_SUCCESS=${BASH_SUCCESS:-"\e[32m[SUCCESS]\e[0m"}
BASH_WARNING=${BASH_WARNING:-"\e[33m[WARNING]\e[0m"}
BASH_ERROR=${BASH_ERROR:-"\e[31m[ERROR]\e[0m"}

# Hokuyo UST-10LX: the LiDAR address comes from the urg_node2 config. Override with env vars if needed.
LIDAR_CONFIG="$REPO_PATH/ws_robot/src/ugv_bringup/config/sensor/lidar_hokuyo_ust10lx.yaml"
LIDAR_IP=${LIDAR_IP:-$(grep -oP "ip_address:\s*'\K[0-9.]+" "$LIDAR_CONFIG")}
LIDAR_IFACE=${LIDAR_IFACE:-$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="ethernet" && $1 !~ /^usb/ {print $1; exit}')}
if [ -z "$LIDAR_HOST_IP" ]; then
    # Same /24 as the LiDAR, host .15 (or .16 if the LiDAR is .15)
    LIDAR_HOST_IP="${LIDAR_IP%.*}.15"
    [ "$LIDAR_HOST_IP" = "$LIDAR_IP" ] && LIDAR_HOST_IP="${LIDAR_IP%.*}.16"
fi
LIDAR_CONNECTION="lidar-ethernet"

echo -e "================================================="
echo -e "$BASH_INFO Checking and linking devices to the system..."
echo -e "$BASH_INFO USB binding using rules from \e[36m$CURRENT_SCRIPT_PATH/usb_rules\e[0m"
echo -e "================================================="

# Copy every rule (devices plugged in later get their names too), then reload once.
RULES_CHANGED=0
for RULE in "$CURRENT_SCRIPT_PATH"/usb_rules/*.rules; do
    if ! cmp -s "$RULE" "/etc/udev/rules.d/$(basename "$RULE")"; then
        echo -e "$BASH_INFO Installing $(basename "$RULE")"
        sudo cp "$RULE" /etc/udev/rules.d/
        RULES_CHANGED=1
    fi
done
if [ $RULES_CHANGED -eq 1 ]; then
    sudo udevadm control --reload-rules
    sudo udevadm trigger --subsystem-match=tty --subsystem-match=input
    sudo udevadm settle
fi

# Serial devices need dialout; joy (SDL) needs input when not on the local desktop session (e.g. over SSH).
for GROUP in dialout input; do
    if ! id -nG "$USER" | grep -qw "$GROUP"; then
        echo -e "$BASH_INFO Adding $USER to the $GROUP group (log out and back in for it to take effect)"
        sudo usermod -aG "$GROUP" "$USER"
    fi
done

if dpkg-query -W -f='${Status}' brltty 2>/dev/null | grep -q "ok installed"; then
    echo -e "$BASH_WARNING brltty is installed which might cause the system unable to detect '\e[36m/dev/ttyUSB*\e[0m'. Please uninstall brltty '\e[36msudo apt remove brltty\e[0m'."
fi
echo -e "================================================="

check_device() {
    local DEVICE_RULE=$1
    local DEVICE_DESC=$2

    local ATTRS_ID_VENDOR=$(grep -o 'ATTRS{idVendor}=="[^"]*"' $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE | cut -d'"' -f2)     # Get vendor id from rules file
    local ATTRS_ID_PRODUCT=$(grep -o 'ATTRS{idProduct}=="[^"]*"' $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE | cut -d'"' -f2)   # Get product id from rules file
    local DEVICE_SYMLINK=$(grep -o 'SYMLINK+="[^"]*"' $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE | cut -d'"' -f2)   # Get symlink from rules file

    if lsusb | grep -q "$ATTRS_ID_VENDOR:$ATTRS_ID_PRODUCT"; then # Check in lsusb if the device with vendor and product id is connected
        if [ -e "/dev/$DEVICE_SYMLINK" ]; then
            echo -e "$BASH_SUCCESS $DEVICE_DESC: found \e[36m/dev/$DEVICE_SYMLINK\e[0m -> $(readlink -f /dev/$DEVICE_SYMLINK)"
        else
            echo -e "$BASH_ERROR $DEVICE_DESC is connected but /dev/$DEVICE_SYMLINK is not found. Try replug in the $DEVICE_DESC and rerun this script."
        fi
    else
        echo -e "$BASH_WARNING $DEVICE_DESC is not connected ($ATTRS_ID_VENDOR:$ATTRS_ID_PRODUCT). If it is, check 'lsusb' against \e[36m$CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE\e[0m."
    fi
}

setup_ethernet_lidar() {
    local DEVICE_DESC=$1

    if [ -z "$LIDAR_IFACE" ] || [ -z "$LIDAR_IP" ]; then
        echo -e "$BASH_ERROR $DEVICE_DESC: no Ethernet interface or LiDAR IP found. Set LIDAR_IFACE / LIDAR_IP."
        return
    fi
    # Dedicated NetworkManager profile with a static address, preferred over any DHCP profile on this port
    if ! nmcli -t -f NAME connection show | grep -qx "$LIDAR_CONNECTION"; then
        echo -e "$BASH_INFO Creating NetworkManager connection '$LIDAR_CONNECTION' on $LIDAR_IFACE ($LIDAR_HOST_IP/24)"
        sudo nmcli connection add type ethernet con-name "$LIDAR_CONNECTION" ifname "$LIDAR_IFACE" \
            ipv4.method manual ipv4.addresses "$LIDAR_HOST_IP/24" ipv4.never-default yes \
            ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 10 >/dev/null
    else
        sudo nmcli connection modify "$LIDAR_CONNECTION" connection.interface-name "$LIDAR_IFACE" \
            ipv4.method manual ipv4.addresses "$LIDAR_HOST_IP/24" ipv4.never-default yes
        sudo nmcli device reapply "$LIDAR_IFACE" >/dev/null 2>&1
    fi
    if ! nmcli -t -f GENERAL.CONNECTION device show "$LIDAR_IFACE" | grep -qx "GENERAL.CONNECTION:$LIDAR_CONNECTION"; then
        sudo nmcli connection up "$LIDAR_CONNECTION" >/dev/null
    fi

    if ping -c 2 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
        echo -e "$BASH_SUCCESS $DEVICE_DESC: reachable at \e[36m$LIDAR_IP\e[0m via $LIDAR_IFACE ($LIDAR_HOST_IP)"
    else
        echo -e "$BASH_ERROR $DEVICE_DESC: no reply from $LIDAR_IP on $LIDAR_IFACE. Check the cable/power, or the LiDAR's IP in $LIDAR_CONFIG."
    fi
}

# VESC USB interface
check_device "99-vesc-usb.rules" "VESC"

# Microsoft XBox One Joypad Joypad USB interface (If using Jetpack 6, see pkg/_tools/pkg_list.bash and uncomment the bash commands to install the driver)
check_device "99-joypad-microsoft-xbox-one-usb.rules" "Microsoft XBox One Joypad"

# Logitech F710 Joypad USB interface (If using Jetpack 6, see pkg/_tools/pkg_list.bash and uncomment the bash commands to install the driver)
check_device "99-joypad-logitech-f710-usb.rules" "Logitech F710 Joypad"

# HiWonder IMU USB interface
check_device "99-imu-hiwonder-usb.rules" "HiWonder IMU"

# LDS01 Lidar USB interface
check_device "99-lidar-robotis-lds01-usb.rules" "LDS01 LiDAR"

# Hokuyo UST-10LX LiDAR Ethernet interface
setup_ethernet_lidar "Hokuyo UST-10LX LiDAR"

echo -e "================================================="
echo -e "$BASH_INFO Interface setup finished."
