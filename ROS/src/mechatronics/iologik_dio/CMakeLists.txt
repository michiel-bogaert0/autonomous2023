cmake_minimum_required(VERSION 3.5.1)
project(iologik_dio)

# C++ 11
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    rosparam_shortcuts
    node_fixture
)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        std_msgs
        rosparam_shortcuts
        node_fixture
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)
link_directories(${CATKIN_DEVEL_PREFIX}/lib)

file(GLOB srcs src/*.cpp include/*.h*)
file(GLOB mxio lib/libmxio_x64.so)

add_executable(${PROJECT_NAME}
  src/iologik_driver.cpp
  src/run.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

target_link_libraries(iologik_dio ${CMAKE_THREAD_LIBS_INIT} ${mxio})