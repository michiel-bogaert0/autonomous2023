cmake_minimum_required(VERSION 3.0.2)
project(slam_controller)

find_package(catkin REQUIRED genmsg rospy ugr_msgs geometry_msgs sensor_msgs fs_msgs)

add_service_files(DIRECTORY srv FILES GetMap.srv SetMap.srv)
generate_messages(DEPENDENCIES ugr_msgs)

catkin_package()

include_directories()