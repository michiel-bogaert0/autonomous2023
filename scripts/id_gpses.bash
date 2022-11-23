#!/bin/bash
R="1-4"
L="1-2"

# echo USB Devices:
# ls /dev/ttyUSB* | cut -d "/" -f 3

for usb in $(ls /dev/ttyUSB* | cut -d "/" -f 3)
do
cd -P /sys/class/tty/$usb/device
path=$(pwd)
if [[ $path =~ $R ]]; then
  echo Right: $usb
fi
if [[ $path =~ $L ]]; then
  echo Left: $usb
fi
done
