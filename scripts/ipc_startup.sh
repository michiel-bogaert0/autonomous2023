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

bash /home/ugentracing/autonomous2023/scripts/id_gpses.bash

exit 0
