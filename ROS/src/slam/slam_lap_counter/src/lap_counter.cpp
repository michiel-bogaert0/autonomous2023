#include "lap_counter.hpp"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "std_msgs/UInt16.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include <node_fixture/node_fixture.hpp>
#include <tf2_ros/transform_listener.h>

namespace slam {
LapCounter::LapCounter(ros::NodeHandle &n)
    : ManagedNode(n, "lap_counter"), n(n), tfListener(tfBuffer) {}

void LapCounter::doConfigure() {
  this->base_link_frame =
      n.param<std::string>("base_link_frame", "ugr/car_base_link");
  this->world_frame = n.param<std::string>("world_frame", "ugr/map");

  std::vector<float> point;
  n.param<std::vector<float>>("finishpoint", point, {0, 0, 0});

  this->finishPoint.x = point[0];
  this->finishPoint.y = point[1];
  this->finishPoint.z = point[2];
  isLoopingTarget = false;

  diagnosticPublisher = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n, "SLAM LC"));
  loopClosedPublisher =
      n.advertise<std_msgs::UInt16>("/output/lapComplete", 5, true);
  reset_service = n.advertiseService(
      "/reset_closure", &slam::LapCounter::handleResetClosureService, this);
  adjust_finish = n.advertiseService(
      "/adjust_finish", &slam::LapCounter::handleAdjustFinishLine, this);
  adjust_targetpoint = n.advertiseService(
      "/adjust_targetpoint", &slam::LapCounter::handleAdjustTargetPoint, this);

  ResetClosure();

  ROS_INFO("Loop closure detection started!");
}

void LapCounter::active() {

  if (this->latestTime - ros::Time::now().toSec() > 0.5 &&
      this->latestTime > 0.0) {
    ROS_WARN("Time went backwards! Resetting...");
    ResetClosure();
    return;
  }
  this->latestTime = ros::Time::now().toSec();

  geometry_msgs::TransformStamped car_pose;
  try {
    car_pose =
        tfBuffer.lookupTransform(this->world_frame, this->base_link_frame,
                                 ros::Time(0), ros::Duration(0.1));
  } catch (const std::exception &e) {
    ROS_ERROR("Car pose lookup failed: %s", e.what());
    return;
  }

  if (!runLoopClosureDetection) {
    return;
  }

  if (firstLap) {
    firstLap = false;

    startPosition.x = car_pose.transform.translation.x;
    startPosition.y = car_pose.transform.translation.y;
    startPosition.z = car_pose.transform.translation.z;
    
    return;
  }

  // ros::ServiceClient client =
  // n.serviceClient<slam_lap_counter::FinishPoint>("/adjust_targetpoint");

  // Create a request message and populate the parameters
  slam_lap_counter::FinishPoint req;
  req.request.point = startPosition;

  // calculate the vector from startpos to current pos
  geometry_msgs::Point distanceVectorToStart;
  distanceVectorToStart.x =
      (float)car_pose.transform.translation.x - startPosition.x;
  distanceVectorToStart.y =
      (float)car_pose.transform.translation.y - startPosition.y;
  distanceVectorToStart.z =
      (float)car_pose.transform.translation.z - startPosition.z;

  geometry_msgs::Point distanceVectorToTarget;
  distanceVectorToTarget.x =
      (float)car_pose.transform.translation.x - targetPoint.x;
  distanceVectorToTarget.y =
      (float)car_pose.transform.translation.y - targetPoint.y;
  distanceVectorToTarget.z =
      (float)car_pose.transform.translation.z - targetPoint.z;

  geometry_msgs::Point distanceVectorToFinish;
  distanceVectorToFinish.x =
      (float)car_pose.transform.translation.x - finishPoint.x;
  distanceVectorToFinish.y =
      (float)car_pose.transform.translation.y - finishPoint.y;
  distanceVectorToFinish.z =
      (float)car_pose.transform.translation.z - finishPoint.z;

  // when the currentpos is close enough to startpos (after racing the round)
  if (checkDistanceGoingUpWhenInRange) {
    // check if the car is in front of the startposition
    // determine if it is the target/finish or start
    if (DotProduct(directionWhenGoingInRange, distanceVectorToStart) < 0 &&
        !isLoopingTarget) {
      amountOfLaps++;
      this->publish();
      ResetLoop();
    } else if (DotProduct(directionWhenGoingInRange, distanceVectorToTarget) <
               0) {
      amountOfLaps++;
      isLoopingTarget = true;
      this->publish();
      ResetLoop();
    } else if (DotProduct(directionWhenGoingInRange, distanceVectorToFinish) <
                   0 &&
               isLoopingTarget) {
      amountOfLaps++;
      runLoopClosureDetection = false;
      this->publish();
    }
    return;
  }
  // init startposition(just te be sure that it isn't {0,0})
  if (startPosition.x == FLT_MAX || startPosition.y == FLT_MAX ||
      startPosition.z == FLT_MAX) {
    startPosition.x = car_pose.transform.translation.x;
    startPosition.y = car_pose.transform.translation.y;
    startPosition.z = car_pose.transform.translation.z;
    if (finishPoint.x == FLT_MAX || finishPoint.y == FLT_MAX ||
        finishPoint.z == FLT_MAX)
      finishPoint = startPosition;
  }

  // calculate distance
  // if the car is outside the outer range -> enable to check if the car is
  // going towards the start position
  if (!doNotCheckDistance) {
    float currentDistanceSquare = 0;
    if (isLoopingTarget && amountOfLaps != 0)
      currentDistanceSquare =
          DotProduct(distanceVectorToTarget, distanceVectorToTarget);
    else
      currentDistanceSquare =
          DotProduct(distanceVectorToStart, distanceVectorToStart);

    if (isinf(currentDistanceSquare))
      return;
    if (maxDistanceSquare < currentDistanceSquare) {
      doNotCheckDistance = true;
    }
  } else {
    float currentDistanceSquareFinish = 0;
    geometry_msgs::Point direction = distanceVectorToFinish;
    if (isLoopingTarget) {
      currentDistanceSquareFinish =
          DotProduct(distanceVectorToTarget, distanceVectorToTarget);
      direction = distanceVectorToTarget;
    } else
      currentDistanceSquareFinish =
          DotProduct(distanceVectorToFinish, distanceVectorToFinish);
    // if the car is close enough to the finishposition
    if (currentDistanceSquareFinish <
        minDistanceForClose * minDistanceForClose) {
      checkDistanceGoingUpWhenInRange = true;
      directionWhenGoingInRange = direction;
    }
  }
}
float LapCounter::DotProduct(const geometry_msgs::Point &a,
                             const geometry_msgs::Point &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
void LapCounter::ResetLoop() {
  doNotCheckDistance = false;
  checkDistanceGoingUpWhenInRange = false;
}

void LapCounter::ResetClosure() {
  ros::Rate loop_rate(50);

  ROS_INFO("Resetting counter");

  startPosition = geometry_msgs::Point();
  startPosition.x = 0.0;
  startPosition.y = 0.0;
  startPosition.z = 0.0;

  doNotCheckDistance = false;
  checkDistanceGoingUpWhenInRange = false;
  directionWhenGoingInRange = geometry_msgs::Point();
  amountOfLaps = 0;

  firstLap = true;

  this->latestTime = ros::Time::now().toSec();

  this->publish();

  runLoopClosureDetection = true;
}

void LapCounter::publish() {

  if (!this->isActive()) {
    return;
  }
  std_msgs::UInt16 msg1;
  msg1.data = amountOfLaps;
  loopClosedPublisher.publish(msg1);
  diagnosticPublisher->publishDiagnostic(
      node_fixture::DiagnosticStatusEnum::OK, "Loop Closure count",
      "#laps: " + std::to_string(amountOfLaps));
}

bool LapCounter::handleResetClosureService(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  if (this->isActive()) {
    return false;
  }

  this->ResetClosure();
  return true;
}

bool LapCounter::handleAdjustFinishLine(
    slam_lap_counter::FinishPoint::Request &request,
    slam_lap_counter::FinishPoint::Response &response) {
  ROS_INFO("Finish line is changed");
  this->finishPoint = request.point;
  isLoopingTarget = false;
  return true;
}

bool LapCounter::handleAdjustTargetPoint(
    slam_lap_counter::FinishPoint::Request &request,
    slam_lap_counter::FinishPoint::Response &response) {
  ROS_INFO("Target point is changed");
  this->targetPoint = request.point;
  isLoopingTarget = true;
  return true;
}
} // namespace slam