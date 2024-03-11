cmake_minimum_required(VERSION 3.0.0)
project(slam_fastslam)

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
  slam_controller
  node_fixture
)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros node_fixture
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

include_directories( /usr/include/eigen3 )

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
endif (CMAKE_COMPILER_IS_GNUCXX)

file(GLOB srcs src/*.cpp include/*.h*)

add_executable(fastslam1 ${srcs})

target_link_libraries(fastslam1
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )