#include "managed_node.hpp"
#include <ugr_msgs/State.h>

namespace node_fixture {

ManagedNode::ManagedNode(ros::NodeHandle &n, std::string name)
    : n(n), name(name) {

  // Create publisher and service
  this->statePublisher = n.advertise<ugr_msgs::State>("/state", 1, true);
  this->serviceServer_set =
      n.advertiseService("/node_managing/" + name + "/set",
                         &ManagedNode::handleSetStateService, this);
  this->serviceServer_get =
      n.advertiseService("/node_managing/" + name + "/get",
                         &ManagedNode::handleGetStateService, this);

  this->state = Unconfigured;
}

bool ManagedNode::handleGetStateService(GetNodeStateRequest &req,
                                        GetNodeStateResponse &res) {
  res.state = ManagedNodeStateStrings[this->state];
  return true;
}

bool ManagedNode::handleSetStateService(SetNodeStateRequest &req,
                                        SetNodeStateResponse &res) {
  if (this->state == Unconfigured &&
      req.state == ManagedNodeStateStrings[Inactive]) {
    this->doConfigure();
    this->state = Inactive;
  } else if (this->state == Inactive &&
             req.state == ManagedNodeStateStrings[Unconfigured]) {
    this->doCleanup();
    this->state = Unconfigured;
  } else if (this->state == Inactive &&
             req.state == ManagedNodeStateStrings[Active]) {
    this->doActivate();
    this->state = Active;
  } else if (this->state == Active &&
             req.state == ManagedNodeStateStrings[Inactive]) {
    this->doDeactivate();
    this->state = Inactive;
  } else if (this->state == Inactive &&
             req.state == ManagedNodeStateStrings[Finalized]) {
    this->doShutdown();
    this->state = Finalized;
  } else if (this->state == Active &&
             req.state == ManagedNodeStateStrings[Finalized]) {
    this->doShutdown();
    this->state = Finalized;
  } else if (this->state == Unconfigured &&
             req.state == ManagedNodeStateStrings[Finalized]) {
    this->doShutdown();
    this->state = Finalized;
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

  res.succes = true;

  // The finalized state should make the node exit.
  if (this->state == Finalized) {
    ros::shutdown();
    exit(0);
  }

  return true;
}

} // namespace node_fixture