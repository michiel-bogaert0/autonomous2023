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
    output+="export MOVING_ROVER_USB=/dev/$usb\n"
    echo "Right (moving rover): $usb"
  fi
  if [[ $path =~ $L ]]; then
    output+="export FIXED_ROVER_USB=/dev/$usb\n"
    echo "Left (fixed rover): $usb"
  fi
done

sudo echo -e "#!/bin/bash\n\n$output" > /home/ugentracing/autonomous2023/env-vars.sh
chmod +x /home/ugentracing/autonomous2023/env-vars.sh

echo "Environment variable export script written to /home/ugentracing/autonomous2023/env-vars.sh"
