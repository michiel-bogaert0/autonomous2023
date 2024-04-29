#include "ethercat_driver.hpp"

ECatDriver::ECatDriver(ros::NodeHandle &n)
    : node_fixture::ManagedNode(n, "ecat_driver"), n(n),
      mode(static_cast<operational_mode_t>(n.param<int>("mode", CSP))),
      ifname(n.param<std::string>("ifname", "enp0s1")) {}

void ECatDriver::doConfigure() {
  // Configure servo and bring in Operational state
  initialize_ethercat(this->ifname.c_str(), this->mode);
  // Start the loop
  start_loop(this->mode);
}

void ECatDriver::doCleanup() {
  // Take down both threads if active
  if (*check_flag || *loop_flag) {
    stop_loop();
  }
  // Reset servo to Initial state
  reset_state();
}

void ECatDriver::doActivate() {
  // Enable servo to follow target position
  *enable_servo = true;
}

void ECatDriver::doDeactivate() {
  // Deactivate servo, but keep in Operational state
  *enable_servo = false;
}

void ECatDriver::doShutdown() {
  // Take down threads
  stop_loop();
  // Reset servo to Initial state
  reset_state();
}
