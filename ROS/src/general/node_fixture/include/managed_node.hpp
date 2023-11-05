#ifndef ROSCPP_MANAGED_NODE_H
#define ROSCPP_MANAGED_NODE_H

#include "node_fixture/SetNodeState.h"
#include "node_fixture/node_fixture.hpp"
#include <ros/ros.h>

namespace node_fixture {
class ManagedNode {
public:
  explicit ManagedNode(ros::NodeHandle &n, std::string name);

  void doConfigure() {}
  void doCleanup() {}
  void doActivate() {}
  void doDeactivate() {}
  void doShutdown() {}

private:
  bool handleSetStateService(node_fixture::SetNodeStateRequest &,
                             node_fixture::SetNodeStateResponse &);

  ros::NodeHandle &n;
  std::string name;

  ros::Publisher statePublisher;
  ros::ServiceServer serviceServer;

  ManagedNodeState state;
};

} // namespace node_fixture

#endif // ROSCPP_MANAGED_NODE_H