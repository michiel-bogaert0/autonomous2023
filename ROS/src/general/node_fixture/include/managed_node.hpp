#ifndef ROSCPP_MANAGED_NODE_H
#define ROSCPP_MANAGED_NODE_H

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
  explicit ManagedNode(ros::NodeHandle &n, std::string name);

  /**
   * \brief functions that are called during state transitions. See
   * https://design.ros2.org/articles/node_lifecycle.html
   */
  void doConfigure(){};
  virtual void doCleanup(){};
  virtual void doActivate(){};
  virtual void doDeactivate(){};
  virtual void doShutdown(){};

  /**
   * \brief Returns current state
   */
  ManagedNodeState getState() { return this->state; }

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
  ros::ServiceServer serviceServer_set;
  ros::ServiceServer serviceServer_get;

  ManagedNodeState state;
};

} // namespace node_fixture

#endif // ROSCPP_MANAGED_NODE_H