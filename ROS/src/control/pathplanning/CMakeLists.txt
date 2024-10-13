cmake_minimum_required(VERSION 3.5.1)
project(pathplanning)

# C++ 11
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    trajectory_msgs
    visualization_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    std_msgs
    rosparam_shortcuts
    ugr_msgs
    tf2_geometry_msgs
    node_fixture
)

find_package(Boost REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        trajectory_msgs
        visualization_msgs
        geometry_msgs
        nav_msgs
        tf2_ros
        std_msgs
        rosparam_shortcuts
        ugr_msgs
        tf2_geometry_msgs
        node_fixture
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

add_executable(${PROJECT_NAME}
  src/center_points.cpp
  src/delaunay.cpp
  src/pathplanning.cpp
  src/paths.cpp
  src/run.cpp
  src/triangulator.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )
