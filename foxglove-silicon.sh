#!/bin/zsh
source ~/.zshrc

# Foxglove bridge
roslaunch foxglove_bridge foxglove_bridge.launch &

# Visualisation of cone observations in Foxglove
roslaunch ~/autonomous2023/ROS/src/perception/launch/perception_process_vis.launch &

# Wait for all background processes to finish
wait