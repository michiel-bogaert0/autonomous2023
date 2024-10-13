cmake_minimum_required(VERSION 3.5.1)
project(slam_lap_counter)

find_package(catkin REQUIRED genmsg rospy)

catkin_python_setup()

generate_messages()
catkin_package()

include_directories()