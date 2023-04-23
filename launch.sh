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

LAUNCH_FILE=$1
shift
if [ -z "$LAUNCH_FILE" ]
  then
    echo "Please provide the name of the launch file."
    exit 1
fi

docker ps -q --filter name=ugr-${LAUNCH_FILE//\//-}-* | xargs docker rm -f

docker run $detach_mode --name ugr-${LAUNCH_FILE//\//-}-$(date +'%Y%m%d%H%M%S') \
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
    -v ./../autonomous2023_binaries:/home/ugr/autonomous2023_binaries/:rw \
    -v /dev:/dev \
    -v ./../rosbags:/home/ugr/rosbags/:rw \
    ugr-base \
    /bin/zsh -c "source ~/.zshrc && roslaunch /home/ugr/autonomous2023/ROS/launch/$LAUNCH_FILE.launch $*"