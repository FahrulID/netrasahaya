#!/bin/bash

WAIT_HOTSPOT="wait-until-hotspot-on.sh"

SCRIPT_DIR="$(dirname "$0")"

SCRIPT_PATH="$SCRIPT_DIR/$WAIT_HOTSPOT"

if [ ! -f "$SCRIPT_PATH" ]; then
	exit 0
fi

bash "$SCRIPT_PATH"
HOTSPOT_STATUS=$?

sleep 5

if [ $HOTSPOT_STATUS -eq 0 ]; then
	echo "localhost"
	exit 0
fi

HOTSPOT_IP=$(nmcli dev show wlan0 | grep 'IP4.ADDRESS' | awk '{print $2}' | cut -d'/' -f1)

if [ -z "$HOTSPOT_IP" ]; then
	echo "localhost"
	exit 0
fi

echo $HOTSPOT_IP
exit 1
