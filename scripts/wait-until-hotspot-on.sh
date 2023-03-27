#!/bin/bash

INTERFACE="wlan0"
MAX_ATTEMPTS=10
ATTEMPTS=0
while [ $ATTEMPTS -lt $MAX_ATTEMPTS ]; do
	if nmcli dev show "$INTERFACE" | grep -q 'Hotspot'; then
		exit 1
	fi
	ATTEMPTS=$(( ATTEMPTS + 1 ))
	sleep 1
done

exit 0
