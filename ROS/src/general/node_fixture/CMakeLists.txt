cmake_minimum_required(VERSION 3.0.2)
project(node_fixture)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_msgs
)

catkin_python_setup()

catkin_package(

  CATKIN_DEPENDS roscpp diagnostic_msgs
  LIBRARIES ${PROJECT_NAME}

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

file(GLOB srcs src/node_fixture.cpp)
add_library(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
