#include "managed_node.hpp"
#include <ugr_msgs/State.h>

namespace node_fixture {

ManagedNode::ManagedNode(ros::NodeHandle &n, std::string name)
    : n(n), name(name) {

  // Create publisher and service
  this->statePublisher = n.advertise<ugr_msgs::State>("/state", 1, true);
  this->serviceServer =
      n.advertiseService("/node_managing/" + name + "/set",
                         &ManagedNode::handleSetStateService, this);

  this->state = Unconfigured;
}

bool ManagedNode::handleSetStateService(
    node_fixture::SetNodeStateRequest &req,
    node_fixture::SetNodeStateResponse &res) {
  if (this->state == Unconfigured && req.state == Inactive) {
    this->doConfigure();
  } else if (this->state == Inactive && req.state == Unconfigured) {
    this->doCleanup();
  } else if (this->state == Inactive && req.state == Active) {
    this->doActivate();
  } else if (this->state == Active && req.state == Inactive) {
    this->doDeactivate();
  } else if (this->state == Inactive && req.state == Finalized) {
    this->doShutdown();
  } else if (this->state == Active && req.state == Finalized) {
    this->doShutdown();
  } else if (this->state == Unconfigured && req.state == Finalized) {
    this->doShutdown();
  } else {
    res.succes = false;
    return true;
  }

  // If state transition is ok, then run the next lines
  ugr_msgs::State stateMsg;
  stateMsg.prev_state = this->state;
  stateMsg.cur_state = req.state;
  stateMsg.scope = this->name;
  stateMsg.header.stamp = ros::Time::now();

  this->statePublisher.publish(stateMsg);

  this->state = req.state;
  res.succes = true;

  // The finalized state should make the node exit.
  if (this->state == Finalized) {
    ros::shutdown();
    exit(0);
  }

  return true;
}

} // namespace node_fixture