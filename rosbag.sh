#!/bin/bash

detach_mode=""

while getopts ":d" opt; do
  case ${opt} in
    d ) detach_mode="-d" ;;
    \? ) echo "Invalid option: -$OPTARG" 1>&2
         exit 1 ;;
  esac
done
shift $((OPTIND -1))

BAG_FILE=$1
shift
if [ -z "$BAG_FILE" ]
  then
    echo "Please provide the name of the bag file."
    exit 1
fi

docker run $detach_mode --name ugr-bag-player-$(date +'%Y%m%d%H%M%S') \
    --privileged \
    --network host \
    -e ROS_HOSTNAME=$ROS_HOSTNAME \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --runtime=nvidia \
    --restart=always \
    -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ./:/home/ugr/autonomous2023/:rw \
    -v ./../rosbags:/home/ugr/rosbags/:rw \
    ugr-base \
    /bin/zsh -c "source ~/.zshrc && rosbag play /home/ugr/rosbags/$BAG_FILE.bag $*"