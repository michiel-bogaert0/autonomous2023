cmake_minimum_required(VERSION 3.5.1)
project(slam_simulator)

find_package(catkin REQUIRED rospy ugr_msgs message_generation fs_msgs)

add_service_files(DIRECTORY srv FILES SetTimeScaler.srv GetGTMap.srv)
generate_messages(DEPENDENCIES fs_msgs)

catkin_package()

include_directories()