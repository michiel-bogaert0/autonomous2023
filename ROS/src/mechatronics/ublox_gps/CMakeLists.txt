cmake_minimum_required(VERSION 2.8.3)
project(ublox_gps)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a catkin package
catkin_package()

add_executable(ntripclient src/run.cpp)
target_link_libraries(ntripclient ${catkin_LIBRARIES})