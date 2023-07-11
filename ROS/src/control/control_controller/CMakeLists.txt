cmake_minimum_required(VERSION 3.0.2)
project(control_controller)

find_package(catkin REQUIRED genmsg rospy ugr_msgs geometry_msgs sensor_msgs fs_msgs)

catkin_python_setup()

catkin_package()

include_directories()