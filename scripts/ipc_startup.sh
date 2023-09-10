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

# Setup environment and start code
sudo /home/$(whoami)/autonomous2023/run set-car pegasus
sudo /home/$(whoami)/autonomous2023/run set-env ROS_HOSTNAME 192.168.50.17
sudo /home/$(whoami)/autonomous2023/run set-env ROS_MASTER_URI http://192.168.50.17:11311

sudo /home/$(whoami)/autonomous2023/run stop
sudo /home/$(whoami)/autonomous2023/run start-headless
sudo /home/$(whoami)/autonomous2023/run pegasus

exit 0
