cmake_minimum_required(VERSION 3.5.1)
project(mpc)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_python_setup()
catkin_package(

)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

