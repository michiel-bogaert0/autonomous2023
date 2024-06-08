#!/bin/bash

current_date_time=$(date +"%Y-%m-%d-%H-%M-%S")
rosbag record -a -O $current_date_time

read -p "Specify reason for rosbag recording: " reason

filtered_bag_name=$current_date_time"_"$reason.bag

# Filter bag
rosbag filter $current_date_time.bag $filtered_bag_name "topic not in ['/ugr/car/sensors/cam0/image']"

rm $current_date_time.bag