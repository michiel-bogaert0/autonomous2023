cmake_minimum_required(VERSION 3.0.0)
project(slam_graphslam)

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

set(g2o_DIR /home/ugr/g2o)
set(g2o_INCLUDE_DIRS ${g2o_DIR} ${g2o_DIR}/build)
set(g2o_BINARY_DIR ${g2o_DIR}/build/bin)
set(g2o_LIBRARY_DIR ${g2o_DIR}/build/lib)

# From other implementation
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse g2o_types_slam2d_addons)

# Find all g2o libraries
set(g2o_LIBRARIES "")

foreach(LIB ${G2O_LIBS})
  # Find library
  find_library(g2o_PATH_${LIB} ${LIB} ${g2o_LIBRARY_DIR})

  # Check if the library is found
  if(g2o_PATH_${LIB})
      # Add the library path to the list
      list(APPEND g2o_LIBRARIES ${g2o_PATH_${LIB}})
  else()
      message(WARNING "Library ${LIB} not found")
  endif()
endforeach()

# find_library(g2o_LIBRARIES
#   NAMES ${G2O_LIBS}
#   PATHS ${g2o_LIBRARY_DIR}
#   REQUIRED
# )

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros node_fixture
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${g2o_INCLUDE_DIRS}
)

include_directories( /usr/include/eigen3 )

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
endif (CMAKE_COMPILER_IS_GNUCXX)

file(GLOB srcs src/*.cpp include/*.h*)

add_executable(graphslam ${srcs})

target_link_libraries(graphslam
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${g2o_LIBRARIES}
  )