cmake_minimum_required(VERSION 3.0.2)
project(minimum_curvature)


find_package(catkin REQUIRED COMPONENTS
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

catkin_package(
    CATKIN_DEPENDS
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
catkin_python_setup()

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

include_directories(

  ${catkin_INCLUDE_DIRS}
)