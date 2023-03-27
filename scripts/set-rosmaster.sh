#!/bin/bash

IP=$(bash /home/jetson/netrasahaya/scripts/get-ip.sh)

export ROS_MASTER_URI="http://$IP:11311"
export ROS_IP="$IP"
export ROS_HOST_NAME="$IP"
