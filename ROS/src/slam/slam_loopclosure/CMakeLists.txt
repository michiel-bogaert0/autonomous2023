cmake_minimum_required(VERSION 3.0.0)
project(slam_loopclosure)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf2_ros
  std_srvs
  geometry_msgs
  diagnostic_msgs
)
find_package(Boost REQUIRED)

catkin_package(

  CATKIN_DEPENDS geometry_msgs roscpp std_srvs tf2 nav_msgs tf2_ros diagnostic_msgs

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
endif (CMAKE_COMPILER_IS_GNUCXX)

file(GLOB srcs src/loopclosure.cpp src/run_loopclosure.cpp include/loopclosure.hpp)

add_executable(loopclosure ${srcs})
target_link_libraries(loopclosure ${catkin_LIBRARIES})