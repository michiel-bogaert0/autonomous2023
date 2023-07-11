include_directories(
  include/
)

# Sim Hardware Interface
add_library(sim_hw_interface
  src/sim_hw_interface.cpp
  src/bicycle_model.cpp
)
target_link_libraries(sim_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(sim_hw_main src/sim_hw_main.cpp)
target_link_libraries(sim_hw_main
  sim_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    sim_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    sim_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/sim_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
