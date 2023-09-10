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

sudo /home/$(whoami)/autonomous2023/run set-car pegasus
sudo /home/$(whoami)/autonomous2023/run set-env ROS_HOSTNAME 192.168.50.17
sudo /home/$(whoami)/autonomous2023/run set-env ROS_MASTER_URI http://192.168.50.17:11311

chmod +x /home/$(whoami)/autonomous2023/env-vars.sh

echo "Environment variable export script written to /home/$(whoami)/autonomous2023/env-vars.sh"

sudo /home/$(whoami)/autonomous2023/run stop
sudo /home/$(whoami)/autonomous2023/run start-headless
sudo /home/$(whoami)/autonomous2023/run pegasus