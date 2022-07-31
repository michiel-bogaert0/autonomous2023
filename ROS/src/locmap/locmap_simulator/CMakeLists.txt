cmake_minimum_required(VERSION 3.0.2)
project(locmap_simulator)

find_package(catkin REQUIRED rospy genmsg)
add_service_files(DIRECTORY srv FILES Dropout.srv)
generate_messages()
catkin_package()

include_directories()