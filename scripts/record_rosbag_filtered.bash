#!/bin/bash

current_date_time=$(date +"%Y-%m-%d-%H-%M-%S")
rosbag record -a -O $current_date_time

read -p "Give rosbag description (enter if no description): " reason

if [ -z "$reason" ]; then
    filtered_bag_name=$current_date_time
else
filtered_bag_name=$current_date_time"_"$reason
fi

read -p "Filter rosbag? (y/n): " filter

if [ $filter == "n" ]; then
    mv $current_date_time.bag $filtered_bag_name.bag
    exit 0
fi

# Filter bag
rosbag filter $current_date_time.bag $filtered_bag_name"_filtered.bag" "topic not in ['/ugr/car/sensors/cam0/image']"

read -p "Do you want to delete the original rosbag? (y/n): " delete_original

if [ $delete_original == "y" ]; then
    rm $current_date_time.bag
fi