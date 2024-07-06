cmake_minimum_required(VERSION 3.5.1)
project(inspection)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package(CATKIN_DEPENDS
    rospy
)

include_directories()
