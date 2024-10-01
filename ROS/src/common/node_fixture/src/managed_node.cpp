#include "managed_node.hpp"
#include <ugr_msgs/State.h>
#include "diagnostic_msgs/DiagnosticStatus.h"

namespace node_fixture {

ManagedNode::ManagedNode(ros::NodeHandle &n, std::string name) : n(n), name(name), healthrate(ros::Rate((double) n.param<float>("healthrate", 3.0))), looprate(ros::Rate((double) n.param<float>("rate", 50.0))) {

  // Create publisher and service
  this->healthPublisher =
      n.advertise<diagnostic_msgs::DiagnosticStatus>("/health/nodes", 1, false);
  this->statePublisher = n.advertise<ugr_msgs::State>("/state", 1, true);
  this->serviceServer_set =
      n.advertiseService("/node_managing/" + name + "/set",
                         &ManagedNode::handleSetStateService, this);
  this->serviceServer_get =
      n.advertiseService("/node_managing/" + name + "/get",
                         &ManagedNode::handleGetStateService, this);

  this->state = Unconfigured;

  this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::OK, "OK", {});

  n.param<bool>("turn_active", turn_active_, false);
  if(turn_active_){
    this->doConfigure();
    this->doActivate();
    this->state = Active;
  }
}

void ManagedNode::spinOnce() {
  ros::spinOnce();

  if (state == Active)
  {
      active();
  }
  else if (state == Inactive)
  {
      inactive();
  }
  else if (state == Unconfigured)
  {
      unconfigured();
  }
  else if (state == Finalized)
  {
      finalized();
  }

  // Publish health
  if ((ros::Time::now() - lastHealthTime).toSec() > healthrate.expectedCycleTime().toSec())
  {
      lastHealthTime = ros::Time::now();
      healthPublisher.publish(this->health);
  }

  this->looprate.sleep();
}

void ManagedNode::setHealthStatus(int level, const std::string &message, const std::vector<diagnostic_msgs::KeyValue> &values) {
    health.level = level;
    health.name = "healthchecks";
    health.hardware_id = name;
    health.message = message;

    diagnostic_msgs::KeyValue kv;
    kv.key = "state";
    kv.value = ManagedNodeStateStrings[this->state];

    health.values.clear();
    health.values.push_back(kv);
    health.values.insert(health.values.end(), values.begin(), values.end());

    healthPublisher.publish(health);
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

  this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::OK, "State changed from " + stateMsg.prev_state + " to " + stateMsg.cur_state, {});

  // The finalized state should make the node exit.
  if (this->state == Finalized) {
    ros::shutdown();
    exit(0);
  }

  return true;
}

} // namespace node_fixture