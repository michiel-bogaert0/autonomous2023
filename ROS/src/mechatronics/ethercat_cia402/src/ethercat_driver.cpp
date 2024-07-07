#include "ethercat_driver.hpp"

ECatDriver::ECatDriver(ros::NodeHandle &n)
    : node_fixture::ManagedNode(n, "ecat_driver"), n(n),
      mode(static_cast<operational_mode_t>(n.param<int>("mode", CSP))),
      ifname(n.param<std::string>("ifname", "enp3s0")) {}

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

  // Initialize publishers
  this->ecat_state_pub = n.advertise<std_msgs::UInt32>("/output/ecat_state", 1);
  this->position_pub = n.advertise<std_msgs::Float32>("/output/position", 1);
  this->statusword_pub = n.advertise<std_msgs::UInt16>("/output/statusword", 1);
  this->velocity_pub = n.advertise<std_msgs::Float32>("/output/velocity", 1);
  this->torque_pub = n.advertise<std_msgs::UInt16>("/output/torque", 1);
  this->erroract_pub = n.advertise<std_msgs::Float32>("/output/erroract", 1);
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

void ECatDriver::set_target(std_msgs::Float32 new_target) {
  // Set new target
  target = static_cast<int32>(new_target.data * RAD_TO_POS);
}

int ECatDriver::update_pubs() {
  int ret = 0;

  // Publish current state
  std_msgs::Int32 ecat_state_msg;
  ecat_state_msg.data = ethercat_state_ext.load();
  this->ecat_state_pub.publish(ecat_state_msg);

  if (ecat_state_msg.data != OP) {
    ret = -1;
  }

  if (mode == CSP) {
    // Read current inputs
    inputs_mutex.lock();
    CSP_inputs inputs = csp_inputs_ext;
    inputs_mutex.unlock();
    std_msgs::Float32 position_msg;
    position_msg.data =
        (static_cast<int32>(inputs.position - base_pos) / RAD_TO_POS);
    this->position_pub.publish(position_msg);

    std_msgs::UInt16 statusword_msg;
    statusword_msg.data = inputs.statusword;
    this->statusword_pub.publish(statusword_msg);

    std_msgs::Float32 velocity_msg;
    velocity_msg.data = (inputs.velocity * TIME_CONV_VEL / RAD_TO_POS);
    this->velocity_pub.publish(velocity_msg);

    std_msgs::UInt16 torque_msg;
    torque_msg.data = inputs.torque;
    this->torque_pub.publish(torque_msg);

    std_msgs::UInt32 erroract_msg;
    erroract_msg.data = (inputs.erroract / RAD_TO_POS);
    this->erroract_pub.publish(erroract_msg);
  } else if (mode == CSV) {
    // Read current inputs
    inputs_mutex.lock();
    CSV_inputs inputs = csv_inputs_ext;
    inputs_mutex.unlock();
    std_msgs::Float32 position_msg;
    position_msg.data =
        static_cast<int32>(inputs.position - base_pos) / RAD_TO_POS;
    this->position_pub.publish(position_msg);

    std_msgs::UInt16 statusword_msg;
    statusword_msg.data = inputs.statusword;
    this->statusword_pub.publish(statusword_msg);

    std_msgs::Float32 velocity_msg;
    velocity_msg.data = (inputs.velocity * TIME_CONV_VEL / RAD_TO_POS);
    this->velocity_pub.publish(velocity_msg);

    std_msgs::UInt16 torque_msg;
    torque_msg.data = inputs.torque;
    this->torque_pub.publish(torque_msg);

    std_msgs::Float32 erroract_msg;
    erroract_msg.data = (inputs.erroract / RAD_TO_POS);
    this->erroract_pub.publish(erroract_msg);
  } else {
    // TODO CST not supported yet
    assert(0 && "Mode not supported");
  }

  return ret;
}

void ECatDriver::active() {
  // Update stats
  int ret = update_pubs();
  if (ret == 0) {
    // Healthy
    setHealthStatus(0, "Active", {});
  } else {
    // Error occurred
    std::string err_msg = "Error occurred (" + std::to_string(ret) + ")";
    setHealthStatus(2, err_msg, {});
  }
}
