#ifndef ECATDRIVER_HPP
#define ECATDRIVER_HPP

#include <string>

#include "ethercat_master.hpp"
#include "managed_node.hpp"
#include "ros/ros.h"
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

private:
  ros::NodeHandle n;
  operational_mode_t mode; // cppcheck-suppress unusedStructMember
  std::string ifname;      // cppcheck-suppress unusedStructMember
  ros::Subscriber target_sub;
};

#endif