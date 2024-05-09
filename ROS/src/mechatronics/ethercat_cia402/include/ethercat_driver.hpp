#ifndef ECATDRIVER_HPP
#define ECATDRIVER_HPP

#include <string>

#include "ethercat_master.hpp"
#include "managed_node.hpp"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"

class ECatDriver : public node_fixture::ManagedNode {
public:
  explicit ECatDriver(ros::NodeHandle &n);
  void doConfigure() override;
  void doCleanup() override;
  void doActivate() override;
  void doDeactivate() override;
  void doShutdown() override;
  void set_target(std_msgs::UInt32 new_target);
  void update_pubs(const ros::TimerEvent &event);

private:
  ros::NodeHandle n;
  operational_mode_t mode; // cppcheck-suppress unusedStructMember
  std::string ifname;      // cppcheck-suppress unusedStructMember

  // Subscriber
  ros::Subscriber target_sub;

  // Publishers
  ros::Timer update_timer;
  double update_period; // cppcheck-suppress unusedStructMember
  ros::Publisher ecat_state_pub;
  ros::Publisher position_pub;
  ros::Publisher statusword_pub;
  ros::Publisher velocity_pub;
  ros::Publisher torque_pub;
  ros::Publisher erroract_pub;
};

#endif