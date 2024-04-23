cmake_minimum_required(VERSION 3.0.0)
project(slam_lap_counter)

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

add_service_files(FILES FinishPoint.srv)

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

file(GLOB srcs src/lap_counter.cpp src/run_lap_counter.cpp include/lap_counter.hpp)



add_executable(lap_counter ${srcs})
add_dependencies(lap_counter ${catkin_EXPORTED_TARGETS})
target_link_libraries(lap_counter ${catkin_LIBRARIES})