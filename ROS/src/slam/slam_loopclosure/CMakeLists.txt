cmake_minimum_required(VERSION 3.0.0)
project(slam_loopclosure)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf2_ros
  std_srvs
  geometry_msgs
  diagnostic_msgs
  message_generation
  message_runtime
  node_fixture
)
find_package(Boost REQUIRED)

add_service_files(DIRECTORY srv FILES FinishPoint.srv)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  roscpp 
  std_srvs 
  tf2 
  nav_msgs 
  tf2_ros
  diagnostic_msgs
  node_fixture
  message_runtime
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
endif (CMAKE_COMPILER_IS_GNUCXX)

file(GLOB srcs src/loopclosure.cpp src/run_loopclosure.cpp include/loopclosure.hpp)

add_executable(loopclosure ${srcs})
target_link_libraries(loopclosure ${catkin_LIBRARIES})