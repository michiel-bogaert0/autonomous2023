cmake_minimum_required(VERSION 3.5.1)
project(parameter_tuner)

find_package(catkin REQUIRED COMPONENTS
    rospy
    node_launcher
)

catkin_package(CATKIN_DEPENDS
    rospy
    node_launcher
)

include_directories()