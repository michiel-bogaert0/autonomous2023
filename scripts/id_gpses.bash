#!/bin/bash
R="1-1"
L="1-2"

# echo USB Devices:
# ls /dev/ttyUSB* | cut -d "/" -f 3

output=""

for usb in $(ls /dev/ttyUSB* | cut -d "/" -f 3)
do
  cd -P /sys/class/tty/$usb/device
  path=$(pwd)
  if [[ $path =~ $R ]]; then
    sudo /home/$(whoami)/autonomous2023/run set-env MOVING_ROVER_USB $usb
    echo "Right (moving rover): $usb"
  fi
  if [[ $path =~ $L ]]; then
    sudo /home/$(whoami)/autonomous2023/run set-env FIXED_ROVER_USB $usb
    echo "Left (fixed rover): $usb"
  fi
done