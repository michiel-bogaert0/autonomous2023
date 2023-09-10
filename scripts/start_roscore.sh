#!/bin/bash

# Get the ROS_MASTER_URI
ros_master_uri="$ROS_MASTER_URI"

# Extract the port number from the URI using awk
port_number=$(echo "$ros_master_uri" | awk -F: '{print $NF}' | tr -d '/')

if [ -n "$port_number" ]; then
  # Run roscore with the extracted port number
  roscore -p "$port_number"
else
  echo "Failed to extract the port number from ROS_MASTER_URI"
  exit 1
fi
