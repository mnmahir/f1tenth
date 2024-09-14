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
    local DEVICE_NAME=$1
    local DEVICE_DESC=$2

    echo -e "$BASH_INFO Binding $DEVICE_DESC"
    local ATTRS_ID_VENDOR=$(grep -o 'ATTRS{idVendor}=="[^"]*"' $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_NAME.rules | cut -d'"' -f2)     # Get vendor id from rules file
    local ATTRS_ID_PRODUCT=$(grep -o 'ATTRS{idProduct}=="[^"]*"' $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_NAME.rules | cut -d'"' -f2)   # Get product id from rules file

    lsusb | grep $ATTRS_ID_VENDOR:$ATTRS_ID_PRODUCT > /dev/null # Check in lsusb if the device with vendor and product id is connected
    if [ $? -eq 0 ]; then
        echo -e "$BASH_INFO $DEVICE_DESC is connected."
        ls /etc/udev/rules.d/ | grep $DEVICE_NAME.rules > /dev/null         # Check if rule exists in /etc/udev/rules.d/
        if [ $? -eq 0 ]; then
            diff $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_NAME.rules /etc/udev/rules.d/$DEVICE_NAME.rules > /dev/null
            if [ $? -eq 0 ]; then
                echo -e "$BASH_INFO $DEVICE_NAME.rules is already exist and is the same."
                cat $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_NAME.rules
            else
                echo -e "$BASH_INFO $DEVICE_NAME.rules is not the same. Copying new..."
                cat $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_NAME.rules
                sudo cp $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_NAME.rules /etc/udev/rules.d
                sudo service udev reload
                sleep 2
                sudo service udev restart
            fi
        else
            echo -e "$BASH_INFO $DEVICE_NAME.rules does not exist. Copying..."
            cat $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_NAME.rules
            sudo cp $CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_NAME.rules /etc/udev/rules.d
            sudo service udev reload
            sleep 2
            sudo service udev restart
        fi
        ls /dev/ | grep $DEVICE_NAME > /dev/null
        if [ $? -eq 0 ]; then
            echo -e "$BASH_SUCCESS found \e[36m/dev/$DEVICE_NAME\e[0m."
        else
            echo -e "$BASH_ERROR /dev/$DEVICE_NAME is not found. Try replug in the $DEVICE_DESC and rerun this script."
            dpkg-query -W -f='${Status}' brltty 2>/dev/null | grep -q "ok installed"
            if [ $? -eq 0 ]; then
                echo -e "$BASH_WARNING brltty is installed which might cause the system unable to detect '\e[36m/dev/ttyUSB*\e[0m'. Please uninstall brltty '\e[36msudo apt remove brltty\e[0m' to bind $DEVICE_DESC."
            fi
        fi
    else
        echo -e "$BASH_WARNING $DEVICE_DESC is not connected. Skipping... "
        echo -e "$BASH_WARNING If connected, type 'lsusb' to check if vendor and product id is correctly defined in \e[36m$CURRENT_SCRIPT_PATH/usb_rules/$DEVICE_NAME.rules\e[0m."
    fi
    echo -e "$BASH_INFO Done with $DEVICE_DESC"
    echo -e "================================================="
}

# VESC USB interface
bind_device "vesc_usb" "VESC"

# Logitech F710 Joypad USB interface
bind_device "joypad_f710_usb" "Logitech F710 Joypad"

# HiWonder IMU USB interface
bind_device "hiwonder_imu_usb" "HiWonder IMU"

# LDS01 Lidar USB interface
bind_device "LDS-01_LiDAR_usb" "LDS01 LiDAR"




echo -e "$BASH_INFO Interface setup finished."