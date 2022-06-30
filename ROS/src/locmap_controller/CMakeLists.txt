cmake_minimum_required(VERSION 3.0.2)
project(locmap_controller)

find_package(catkin REQUIRED genmsg actionlib_msgs rospy ugr_msgs geometry_msgs sensor_msgs fs_msgs)
add_action_files(DIRECTORY action FILES StartSimulation.action)
add_service_files(DIRECTORY srv FILES Launch.srv Stop.srv)
add_message_files(DIRECTORY msg FILES ServiceRequest.msg ServiceResponse.msg)
generate_messages(DEPENDENCIES actionlib_msgs ugr_msgs)

catkin_package()

include_directories()