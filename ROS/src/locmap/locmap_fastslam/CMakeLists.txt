cmake_minimum_required(VERSION 3.0.2)
project(locmap_fastslam)

find_package(catkin REQUIRED rospy)

# The standard catkin way of building python modules is used
catkin_python_setup()

catkin_package()

# This custom target is used to invoke the Cython build.
# When importing the catkin python module it will fallback to the compiled .so files
ADD_CUSTOM_TARGET(do_always ALL COMMAND ${PROJECT_SOURCE_DIR}/cython_setup.sh)

include_directories()