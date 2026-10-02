#!/bin/bash

# Author: Mahir Sehmi
# Description: Makes the car notice within 0.5 s that a Bluetooth game controller is out of range.
#              Until a Bluetooth LE link times out, the controller looks connected and the car keeps acting on
#              the last stick positions it received. The Xbox Wireless Controller asks for a 3 s timeout; this
#              sets the car's side of the link to 0.5 s (the car is the central and decides) and stores it
#              for the next connection. Run as root: udev starts it whenever a controller connects
#              (usb_rules/99-joypad-bluetooth-link.rules), installed by interface_bringup.bash.
#              Usage: f1tenth-joystick-link [timeout_ms]

TIMEOUT_MS=${1:-500}
UNITS=$((TIMEOUT_MS / 10))  # Bluetooth counts the supervision timeout in 10 ms
NAME_PATTERN="Xbox Wireless Controller"

apply() {
    # LE links: "< LE EC:83:50:95:CD:E1 handle 64 state 1 lm CENTRAL AUTH ENCRYPT"
    hcitool con | awk '$2 == "LE" {print $3, $5}' | while read -r addr handle; do
        name=$(bluetoothctl info "$addr" 2>/dev/null | sed -n 's/^\s*Name: //p')
        [[ "$name" == *"$NAME_PATTERN"* ]] || continue
        # Interval 7.5 ms, no latency: what the controller asks for; only the timeout changes
        if hcitool lecup --handle "$handle" --min 6 --max 6 --latency 0 --timeout "$UNITS"; then
            logger -t f1tenth-joystick-link "$name ($addr): link timeout ${TIMEOUT_MS} ms"
        fi
        # Start the next connection with it too (read by bluetoothd at boot)
        for info in /var/lib/bluetooth/*/"$addr"/info; do
            [ -f "$info" ] && sed -i "/^\[ConnectionParameters\]/,/^\[/ s/^Timeout=.*/Timeout=$UNITS/" "$info"
        done
    done
}

# The controller sets up its own link parameters right after connecting: apply after that, and once more later
sleep 3
apply
sleep 10
apply
