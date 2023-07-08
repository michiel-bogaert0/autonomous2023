include_directories(
  include/
)

# Pegasus Hardware Interface
add_library(pegasus_hw_interface
  src/pegasus_hw_interface.cpp
)
target_link_libraries(pegasus_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(pegasus_hw_main src/pegasus_hw_main.cpp)
target_link_libraries(pegasus_hw_main
  pegasus_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    pegasus_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    pegasus_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/pegasus_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
