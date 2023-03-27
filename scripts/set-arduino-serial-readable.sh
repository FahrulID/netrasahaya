#!/bin/bash

SERIAL=/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_14101-if00

MAX_ATTEMPTS=10
ATTEMPTS=0

while [ ! -e "$SERIAL" ]; do
    if [ $ATTEMPTS -gt $MAX_ATTEMPTS ]; then
        exit 0
    fi

    ATTEMPTS=$(( ATTEMPTS + 1 ))
    sleep 1
done

sudo chmod 777 $SERIAL
exit 1