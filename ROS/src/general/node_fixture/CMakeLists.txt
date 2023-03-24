cmake_minimum_required(VERSION 3.0.2)
project(node_fixture)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_msgs
)

catkin_python_setup()

catkin_package(

  CATKIN_DEPENDS geometry_msgs roscpp diagnostic_msgs

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

file(GLOB src/node_fixture.cpp include/node_fixture.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

