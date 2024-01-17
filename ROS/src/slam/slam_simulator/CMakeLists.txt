cmake_minimum_required(VERSION 3.0.2)
project(slam_simulator)

find_package(catkin REQUIRED rospy ugr_msgs message_generation)

add_service_files(DIRECTORY srv FILES SetTimeScaler.srv)
generate_messages()

catkin_package()

include_directories()