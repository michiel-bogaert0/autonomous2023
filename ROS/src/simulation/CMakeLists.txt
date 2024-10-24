cmake_minimum_required(VERSION 3.5.1)
project(simulation)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
