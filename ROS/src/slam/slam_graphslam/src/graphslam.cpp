// #define _GLIBCXX_USE_CXX11_ABI 0
#include "graphslam.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include "ros/console.h"
#include "ros/ros.h"

#include "ros/service_client.h"
#include <slam_controller/SetMap.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

#include "message_filters/subscriber.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

#include <string>

#include <ugr_msgs/ObservationWithCovariance.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

namespace slam {
GraphSLAM::GraphSLAM(ros::NodeHandle &n)
    : ManagedNode(n, "graphslam"), n(n), tfListener(tfBuffer),
      base_link_frame(n.param<string>("base_link_frame", "ugr/car_base_link")),
      tf2_filter(obs_sub, tfBuffer, base_link_frame, 1, 0) {}

void GraphSLAM::doConfigure() {
  // Initialize frames
  this->slam_base_link_frame =
      n.param<string>("slam_base_link_frame", "ugr/slam_base_link");
  this->world_frame = n.param<string>("world_frame", "ugr/car_odom");
  this->map_frame = n.param<string>("map_frame", "ugr/map");
  this->lidar_frame = n.param<string>("lidar_frame", "os_sensor");

  // Initialize parameters
  this->doSynchronous = n.param<bool>("synchronous", true);
  this->publish_rate = n.param<double>("publish_rate", 3.0);

  // Initialize map Service Client
  string SetMap_service =
      n.param<string>("SetMap_service", "/ugr/srv/slam_map_server/set");
  this->setmap_srv_client =
      n.serviceClient<slam_controller::SetMap::Request>(SetMap_service, true);

  // Initialize map namespace
  this->globalmap_namespace = n.param<string>("globalmap_namespace", "global");
  this->localmap_namespace = n.param<string>("localmap_namespace", "local");

  // Initialize publishers
  this->odomPublisher = n.advertise<nav_msgs::Odometry>("/output/odom", 5);
  this->diagPublisher = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n, "SLAM GrapSLAM"));

  // Initialize subscribers
  obs_sub.subscribe(n, "/input/observations", 1);
  tf2_filter.registerCallback(
      boost::bind(&GraphSLAM::handleObservations, this, _1));

  // Initialize variables
  this->gotFirstObservations = false;
}

void GraphSLAM::handleObservations(
    const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs) {
  // Node lifecycle check
  if (!this->isActive()) {
    return;
  }
  // Time check
  if (this->latestTime - obs->header.stamp.toSec() > 5.0 &&
      this->latestTime > 0.0) {
    // Reset
    ROS_WARN("Time went backwards! Resetting graphslam...");
    this->diagPublisher->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::WARN, "Backwards time",
        "Graphslam reset (time went backwards)!");

    this->latestTime = 0.0;
    return;
  } else {
    this->latestTime = obs->header.stamp.toSec();
  }

  // set variables
  this->observations = *obs;
  this->gotFirstObservations = true;

  if (this->doSynchronous) {
    this->step();
  }
}

void GraphSLAM::step() {
  if (!this->isActive() || !this->gotFirstObservations) {
    return;
  }
}

void GraphSLAM::publishOutput(ros::Time lookupTime) {

  if (std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - this->prev_publish_time)
          .count() < 1.0 / this->publish_rate) {
    return;
  }

  this->prev_publish_time = std::chrono::steady_clock::now();
}
} // namespace slam
