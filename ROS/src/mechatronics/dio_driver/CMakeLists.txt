cmake_minimum_required(VERSION 3.0.0)
project(dio_driver)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  node_fixture
)
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs node_fixture
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)
include_directories( /usr/include/eigen3 )
link_directories(${CATKIN_DEVEL_PREFIX}/lib)
if (CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
endif (CMAKE_COMPILER_IS_GNUCXX)
file(GLOB srcs src/*.cpp include/*.h*)
add_executable(run_driver ${srcs})
target_link_libraries(run_driver
  ${catkin_LIBRARIES}
  ${CMAKE_DL_LIBS}
)