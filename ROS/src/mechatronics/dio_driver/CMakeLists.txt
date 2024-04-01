cmake_minimum_required(VERSION 3.0.0)
project(dio_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ugr_msgs
  node_fixture  
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs node_fixture
)