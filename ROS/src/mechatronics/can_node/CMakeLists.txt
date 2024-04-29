cmake_minimum_required(VERSION 3.5.1)
project(can_node)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
