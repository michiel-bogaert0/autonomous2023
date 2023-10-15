#!/bin/bash

# Check if user has sufficient privileges
if [[ $(id -u) -ne 0 ]]; then
    echo "This script must be run as root"
    exit 1
fi

# Set permissions
sudo chmod 777 /dev/ttyUSB* /dev/ttyACM*

echo "Permissions set for ttyUSB and ttyACM devices"

# Stack startup

cd /home/ugentracing/autonomous2023

/home/ugentracing/autonomous2023/run set-car pegasus
/home/ugentracing/autonomous2023/run set-env ROS_HOSTNAME 192.168.50.17
/home/ugentracing/autonomous2023/run set-env ROS_MASTER_URI http://192.168.50.17:11311

# Start code
/home/ugentracing/autonomous2023/run stop
/home/ugentracing/autonomous2023/run start-headless
/home/ugentracing/autonomous2023/run pegasus

exit 0
