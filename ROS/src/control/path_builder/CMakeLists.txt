cmake_minimum_required(VERSION 3.5.1)
project(path_builder)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
catkin_package(

)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

