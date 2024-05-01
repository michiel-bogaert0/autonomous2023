include_directories(
  include/
)

# Gen4 Hardware Interface
add_library(gen4_hw_interface
  src/gen4_hw_interface.cpp
)
target_link_libraries(gen4_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(gen4_hw_main src/gen4_hw_main.cpp)
target_link_libraries(gen4_hw_main
  gen4_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    gen4_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    gen4_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/gen4_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
