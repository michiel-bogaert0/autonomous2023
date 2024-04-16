cmake_minimum_required(VERSION 3.0.2)
project(ugr_ros_control)

# C++ 11
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  can_msgs
  cmake_modules
  control_msgs
  control_toolbox
  controller_manager
  hardware_interface
  joint_limits_interface
  roscpp
  rosparam_shortcuts
  sensor_msgs
  std_msgs
  tf2_ros
  trajectory_msgs
  transmission_interface
  ugr_msgs
  urdf
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    can_msgs
    control_msgs
    control_toolbox
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    rosparam_shortcuts
    sensor_msgs
    std_msgs
    trajectory_msgs
    transmission_interface
    ugr_msgs
    urdf
  LIBRARIES
    generic_hw_control_loop
    generic_hw_interface
)

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

# Control loop
add_library(generic_hw_control_loop
  src/generic_hw_control_loop.cpp
)
target_link_libraries(generic_hw_control_loop
  ${catkin_LIBRARIES}
)

# Generic Hardware Interface
add_library(generic_hw_interface
  src/generic_hw_interface.cpp
)
target_link_libraries(generic_hw_interface
  ${catkin_LIBRARIES}
)

## TOOLS ------------------------------------------------------

# Tool for controlling a robot from keyboard
add_executable(${PROJECT_NAME}_keyboard_teleop src/tools/keyboard_teleop.cpp)
set_target_properties(${PROJECT_NAME}_keyboard_teleop PROPERTIES OUTPUT_NAME keyboard_teleop PREFIX "")
target_link_libraries(${PROJECT_NAME}_keyboard_teleop
  ${catkin_LIBRARIES}
)

add_subdirectory(sim_control)
add_subdirectory(pegasus_control)
add_subdirectory(gen4_control)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    generic_hw_control_loop
    generic_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    ${PROJECT_NAME}_keyboard_teleop
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
