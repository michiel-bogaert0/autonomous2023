cmake_minimum_required(VERSION 3.0.2)
project(lidar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

include_directories( /usr/include/eigen3 )

add_executable(${PROJECT_NAME}
  src/lidar.cpp
  src/ground_removal.cpp
  src/ground_removal2.cpp
  src/cone_clustering.cpp
  src/run.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  )