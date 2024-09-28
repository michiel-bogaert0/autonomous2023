cmake_minimum_required(VERSION 3.5.1)
project(camera)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  ugr_msgs
  diagnostic_msgs
  cv_bridge
)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZLIB)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

get_filename_component(NEOAPI_ROOT_DIR ${PROJECT_SOURCE_DIR}/include/neoapi ABSOLUTE)
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
if (${ARCHITECTURE} STREQUAL "arm64" OR ${ARCHITECTURE} STREQUAL "aarch64")
  link_directories(${NEOAPI_ROOT_DIR}/lib_aarch64)
else()
  link_directories(${NEOAPI_ROOT_DIR}/lib_x86_64)
endif()

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${YAML_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
  ${NEOAPI_ROOT_DIR}/include
)

include_directories( /usr/include/eigen3 )

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
endif (CMAKE_COMPILER_IS_GNUCXX)

file(GLOB srcs src/*.cpp include/*.h*)

add_executable(cameracpp ${srcs})

find_package(PythonLibs REQUIRED) 
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(cameracpp
	${PYTHON_LIBRARIES}
	${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  neoapi_cpp
  ZLIB::ZLIB
  )