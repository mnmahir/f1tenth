#!/bin/bash

# Author: Mahir Sehmi
# Description: This script is used to bind USB devices to the system.
#              It copies the rules file to /etc/udev/rules.d/ and restarts udev service.
#              It also checks if the device is connected and prints the status.

# get current script path
CURRENT_SCRIPT_PATH=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

echo -e "================================================="
echo -e "$BASH_INFO Checking and linking devices to the system..."
echo -e "$BASH_INFO USB binding using rules from \e[36m$CURRENT_SCRIPT_PATH/usb_rules\e[0m"
echo -e "================================================="

bind_device() {
    local DEVICE_RULE=$1
    local DEVICE_DESC=$2

    echo -e "$BASH_INFO Binding $DEVICE_DESC"
    local ATTRS_ID_VENDOR=$(grep -o 'ATTRS{idVendor}=="[^"]*"' $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE | cut -d'"' -f2)     # Get vendor id from rules file
    local ATTRS_ID_PRODUCT=$(grep -o 'ATTRS{idProduct}=="[^"]*"' $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE | cut -d'"' -f2)   # Get product id from rules file
    local DEVICE_SYMLINK=$(grep -o 'SYMLINK+="[^"]*"' $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE | cut -d'"' -f2)   # Get symlink from rules file
\

    lsusb | grep $ATTRS_ID_VENDOR:$ATTRS_ID_PRODUCT > /dev/null # Check in lsusb if the device with vendor and product id is connected
    if [ $? -eq 0 ]; then
        echo -e "$BASH_INFO $DEVICE_DESC is connected."
        ls /etc/udev/rules.d/ | grep $DEVICE_RULE > /dev/null         # Check if rule exists in /etc/udev/rules.d/
        if [ $? -eq 0 ]; then
            diff $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE /etc/udev/rules.d/$DEVICE_RULE > /dev/null
            if [ $? -eq 0 ]; then
                echo -e "$BASH_INFO $DEVICE_RULE is already exist and is the same."
                cat $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE
            else
                echo -e "$BASH_INFO $DEVICE_RULE is not the same. Copying new..."
                cat $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE
                sudo cp $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE /etc/udev/rules.d
                sudo service udev reload
                sleep 2
                sudo service udev restart
            fi
        else
            echo -e "$BASH_INFO $DEVICE_RULE does not exist. Copying..."
            cat $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE
            sudo cp $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE /etc/udev/rules.d
            sudo service udev reload
            sleep 2
            sudo service udev restart
        fi
        # ls /dev/ | grep $DEVICE_SYMLINK > /dev/null
        if [ -e "/dev/$DEVICE_SYMLINK"  ]; then
            echo -e "$BASH_SUCCESS found \e[36m/dev/$DEVICE_SYMLINK\e[0m."
        else
            echo -e "$BASH_ERROR /dev/$DEVICE_SYMLINK is not found. Try replug in the $DEVICE_DESC and rerun this script."
            dpkg-query -W -f='${Status}' brltty 2>/dev/null | grep -q "ok installed"
            if [ $? -eq 0 ]; then
                echo -e "$BASH_WARNING brltty is installed which might cause the system unable to detect '\e[36m/dev/ttyUSB*\e[0m'. Please uninstall brltty '\e[36msudo apt remove brltty\e[0m' to bind $DEVICE_DESC."
            fi
        fi
    else
        echo -e "$BASH_WARNING $DEVICE_DESC is not connected. Skipping... "
        echo -e "$BASH_WARNING If connected, type 'lsusb' to check if vendor and product id is correctly defined in \e[36m$CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_RULE\e[0m."
    fi
    echo -e "$BASH_INFO Done with $DEVICE_DESC"
    echo -e "================================================="

    



}

# VESC USB interface
bind_device "99-vesc-usb.rules" "VESC"

# Microsoft XBox One Joypad Joypad USB interface
bind_device "99-joypad-microsoft-xbox-one-usb.rules" "Microsoft XBox One Joypad"

# Logitech F710 Joypad USB interface (If using Jetpack 6, see pkg/_tools/pkg_list.bash and uncomment the bash commands to install the driver using 'r2pkg')
bind_device "99-joypad-logitech-f710-usb.rules" "Logitech F710 Joypad"

# HiWonder IMU USB interface
bind_device "99-imu-hiwonder-usb.rules" "HiWonder IMU"

# LDS01 Lidar USB interface
bind_device "99-lidar-lds01-usb.rules" "LDS01 LiDAR"




echo -e "$BASH_INFO Interface setup finished."