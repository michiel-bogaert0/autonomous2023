#ifndef ROSCPP_MANAGED_NODE_H
#define ROSCPP_MANAGED_NODE_H

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "node_fixture/GetNodeState.h"
#include "node_fixture/SetNodeState.h"
#include "node_fixture/node_fixture.hpp"
#include <ros/ros.h>

namespace node_fixture {
class ManagedNode {
public:
  /**
   * \brief Base class to make a managed node
   *
   * Args:
   *  - ros::NodeHandle &n: reference to a node handle
   *  - std::string name: the name of the node.
   */
  ManagedNode(ros::NodeHandle &n, std::string name);

  /**
   * \brief functions that are called during state transitions. See
   * https://design.ros2.org/articles/node_lifecycle.html
   */
  virtual void doConfigure() {}
  virtual void doCleanup() {}
  virtual void doActivate() {}
  virtual void doDeactivate() {}
  virtual void doShutdown() {}
  virtual void active() {}
  virtual void inactive() {}
  virtual void unconfigured() {}
  virtual void finalized() {}

  /**
   * \brief Returns current state
   */
  ManagedNodeState getState() { return this->state; }
  bool isActive() { return this->state == Active; }

  /**
   * \brief Sets the health of the node
   *
   * Args:
   * - int level: the level of the health status
   * - std::string message: the message of the health status
   * - std::vector<diagnostic_msgs::KeyValue> values: the values of the health
   * status
   */
  void setHealthStatus(int level, const std::string &message,
                       const std::vector<diagnostic_msgs::KeyValue> &values);

  /**
   * \brief Spins once.
   *
   * Use this instead of ros::spinOnce() to handle state transitions.
   */
  void spinOnce();

  /**
   * \brief Spins forever.
   *
   * Use this instead of ros::spin() to handle state transitions.
   */
  void spin() {
    while (ros::ok()) {
      this->spinOnce();
    }
  }

  ManagedNodeState state;

private:
  /**
   * \brief handles SetNodeState calls
   */
  bool handleSetStateService(SetNodeStateRequest &, SetNodeStateResponse &);

  /**
   * \brief handles GetNodeState calls
   */
  bool handleGetStateService(GetNodeStateRequest &, GetNodeStateResponse &);

  ros::NodeHandle &n;
  std::string name;

  ros::Publisher statePublisher;
  ros::Publisher healthPublisher;
  ros::ServiceServer serviceServer_set;
  ros::ServiceServer serviceServer_get;

  ros::Rate looprate;
  ros::Rate healthrate;
  ros::Time lastHealthTime;
  diagnostic_msgs::DiagnosticStatus health;
  bool turn_active_;
};

} // namespace node_fixture

#endif // ROSCPP_MANAGED_NODE_H