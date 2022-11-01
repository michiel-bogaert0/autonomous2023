cmake_minimum_required(VERSION 3.0.0)
project(locmap_loopclosure)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  tf2_ros
)
find_package(Boost REQUIRED)

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
endif (CMAKE_COMPILER_IS_GNUCXX)

file(GLOB srcs src/*.cpp include/*.h*)

add_executable(loopclosure ${srcs})
target_link_libraries(loopclosure ${catkin_LIBRARIES})