include_directories(
  include/
)

# Orion Hardware Interface
add_library(orion_hw_interface
  src/orion_hw_interface.cpp
)
target_link_libraries(orion_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(orion_hw_main src/orion_hw_main.cpp)
target_link_libraries(orion_hw_main
  orion_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    orion_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    orion_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/orion_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
