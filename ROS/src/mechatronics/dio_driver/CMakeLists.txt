cmake_minimum_required(VERSION 3.0.0)
project(dio_driver)

# Find required Catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  node_fixture
)

# Declare Catkin package dependencies
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs node_fixture
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  /usr/include/eigen3
)

# Link directories
link_directories(${CATKIN_DEVEL_PREFIX}/lib)

# Set compiler flags
if (CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
endif (CMAKE_COMPILER_IS_GNUCXX)

# Define source files
file(GLOB srcs src/*.cpp include/*.h*)


# Create executable target
add_executable(run_driver ${srcs})

# Link libraries
target_link_libraries(run_driver
  ${catkin_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

# Custom command to copy libvecow.so
add_custom_command(
    TARGET run_driver POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/src/libvecow.so
            ~/.ros/libvecow.so
)

# Custom command to copy IOConfig folder
add_custom_command(
    TARGET run_driver POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/src/IOConfig
            ~/.ros/IOConfig
)
