#!/bin/bash

# Author: Mahir Sehmi
# Description: Adds "F1TENTH Pit Wall" to the applications menu, so the UI starts with a click.
#              Works on the car and on any computer where f1tenth_bringup is built (see the README).
#              Usage: bash scripts/install_ui_shortcut.bash [--remove]

CURRENT_SCRIPT_PATH=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
REPO_PATH=$(dirname "$CURRENT_SCRIPT_PATH")
APP=$HOME/.local/share/applications/f1tenth-pit-wall.desktop
ICON=$HOME/.local/share/icons/hicolor/scalable/apps/f1tenth-pit-wall.svg

if [ "$1" = "--remove" ]; then
    rm -f "$APP" "$ICON"
    echo "Removed the F1TENTH Pit Wall shortcut"
    exit 0
fi

mkdir -p "$(dirname "$APP")" "$(dirname "$ICON")"
cp "$REPO_PATH/ws_robot/src/f1tenth_bringup/resources/f1tenth-pit-wall.svg" "$ICON"
chmod +x "$CURRENT_SCRIPT_PATH/pit_wall.bash"
cat > "$APP" <<DESKTOP_EOF
[Desktop Entry]
Type=Application
Name=F1TENTH Pit Wall
Comment=Drive, map and race the F1TENTH car
Exec=$CURRENT_SCRIPT_PATH/pit_wall.bash
Icon=f1tenth-pit-wall
Terminal=false
Categories=Development;Science;
StartupWMClass=f1tenth_ui
DESKTOP_EOF
update-desktop-database "$(dirname "$APP")" > /dev/null 2>&1 || true
echo "Added 'F1TENTH Pit Wall' to the applications menu (right-click it to add it to the dock)."
