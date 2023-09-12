#!/bin/bash

# Check if user has sufficient privileges
if [[ $(id -u) -ne 0 ]]; then
    echo "This script must be run as root"
    exit 1
fi

# Set permissions
sudo chmod 777 /dev/ttyUSB* /dev/ttyACM*

echo "Permissions set for ttyUSB and ttyACM devices"

# Now recognize left (fixed) GPS and split it with socat

L="1-2"

for usb in $(ls /dev/ttyUSB* | cut -d "/" -f 3)
do
cd -P /sys/class/tty/$usb/device
path=$(pwd)
if [[ $path =~ $L ]]; then
  echo "Left (fixed rover): $usb"
  echo "Splitting to /dev/ttybase"
  sudo socat /dev/ttyUSB1 /dev/ttybase
  sudo chmod 777 /dev/ttybase
fi
done

# Setup environment
R="1-1"
L="1-2"

# echo USB Devices:
ls /dev/ttyUSB* | cut -d "/" -f 3

for usb in $(ls /dev/ttyUSB* | cut -d "/" -f 3)
do
  cd -P /sys/class/tty/$usb/device
  path=$(pwd)
  if [[ $path =~ $R ]]; then
    cd /home/ugentracing/autonomous2023
    /home/ugentracing/autonomous2023/run set-env MOVING_ROVER_USB $usb
    echo "Right (moving rover): $usb"
  fi
  if [[ $path =~ $L ]]; then
    cd /home/ugentracing/autonomous2023
    /home/ugentracing/autonomous2023/run set-env FIXED_ROVER_USB $usb
    echo "Left (fixed rover): $usb"
  fi
done

cd /home/ugentracing/autonomous2023

/home/ugentracing/autonomous2023/run set-car pegasus
/home/ugentracing/autonomous2023/run set-env ROS_HOSTNAME 192.168.50.17
/home/ugentracing/autonomous2023/run set-env ROS_MASTER_URI http://192.168.50.17:11311

# Start code
/home/ugentracing/autonomous2023/run stop
/home/ugentracing/autonomous2023/run start-headless
/home/ugentracing/autonomous2023/run pegasus

exit 0
