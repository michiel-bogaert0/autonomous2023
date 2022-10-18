cmake_minimum_required(VERSION 3.0.0)
project(locmap_loopclosure)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
  ugr_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
endif (CMAKE_COMPILER_IS_GNUCXX)

file(GLOB srcs src/*.cpp include/*.h*)

add_executable(loopclosure ${srcs})
target_link_libraries(loopclosure ${catkin_LIBRARIES})