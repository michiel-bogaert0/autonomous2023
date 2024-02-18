cmake_minimum_required(VERSION 3.0.2)
project(minimum_curvature)


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

catkin_python_setup()
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

  ${catkin_INCLUDE_DIRS}
)