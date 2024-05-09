#include "ethercat_driver.hpp"

ECatDriver::ECatDriver(ros::NodeHandle &n)
    : node_fixture::ManagedNode(n, "ecat_driver"), n(n),
      mode(static_cast<operational_mode_t>(n.param<int>("mode", CSP))),
      ifname(n.param<std::string>("ifname", "enp3s0")) {
  // Manual run
  ROS_INFO("Configuring");
  doConfigure();
  ROS_INFO("Configured");
  ros::Duration(2).sleep();
  ROS_INFO("Activating...");
  doActivate();
  for (int i = 0; i < 20; i++) {
    ROS_INFO("State: %#x", servo_state.statusword_state);
    target += 2048;
    ros::Duration(1).sleep();
  }
  ROS_INFO("Shutting down...");
  doShutdown();
}

void ECatDriver::doConfigure() {
  // Configure servo and bring in Operational state
  initialize_ethercat(this->ifname.c_str(), this->mode);
  // Start the loop
  start_loop(this->mode);
}

void ECatDriver::doCleanup() {
  // Take down both threads if active
  if (check_flag.load() || loop_flag.load()) {
    stop_loop();
  }
  // Reset servo to Initial state
  reset_state();
}

void ECatDriver::doActivate() {
  // Enable servo to follow target position
  enable_servo = true;

  // Start subscribers
  this->target_sub =
      this->n.subscribe("/input/target", 1, &ECatDriver::set_target, this);
}

void ECatDriver::doDeactivate() {
  // Deactivate servo, but keep in Operational state
  enable_servo = false;
}

void ECatDriver::doShutdown() {
  // Take down threads
  stop_loop();
  // Reset servo to Initial state
  reset_state();
}

void ECatDriver::set_target(std_msgs::UInt32 new_target) {
  // Set new target
  target = new_target.data;
}
