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

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include "edge_se2.hpp"
#include "edge_se2_pointxy.hpp"
#include "se2.hpp"
#include "vertex_point_xy.hpp"
#include "vertex_se2.hpp"

using namespace std;
using namespace Eigen;
using namespace g2o;

namespace slam {

GraphSLAM::GraphSLAM(ros::NodeHandle &n)
    : ManagedNode(n, "graphslam"), n(n), tfListener(tfBuffer),
      base_link_frame(n.param<string>("base_link_frame", "ugr/car_base_link")),
      tf2_filter(obs_sub, tfBuffer, base_link_frame, 1, 0) {
  this->doConfigure(); // remove this line
}

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

  this->max_range = 15;
  this->max_half_fov = 60 * 0.0174533;

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
  this->prev_state = {0.0, 0.0, 0.0};
  this->poseIndex = 0;
  this->landmarkIndex = 0;

  //----------------------------------------------------------------------------
  //------------------------------ Set Optimizer -------------------------------
  //----------------------------------------------------------------------------
  typedef BlockSolver<BlockSolverTraits<-1, -1>> SlamBlockSolver;
  typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  auto linearSolver = make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  OptimizationAlgorithmGaussNewton *solver =
      new OptimizationAlgorithmGaussNewton(
          make_unique<SlamBlockSolver>(move(linearSolver)));

  this->optimizer.setAlgorithm(solver);
  //-----------------------------------------------------------------------------
}

void GraphSLAM::handleObservations(
    const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs) {
  // Node lifecycle check
  // if (!this->isActive()) {
  //   return;
  // }
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
  // if (!this->isActive() || !this->gotFirstObservations) {
  if (!this->gotFirstObservations) {
    return;
  }

  // --------------------------------------------------------------------
  // ------------------- Transform observations -------------------------
  // --------------------------------------------------------------------
  // Transform the observations to the base_link frame from their time to the
  // new time

  ugr_msgs::ObservationWithCovarianceArrayStamped transformed_obs;
  transformed_obs.header.frame_id = this->base_link_frame;
  transformed_obs.header.stamp = ros::Time::now();

  for (auto observation : this->observations.observations) {
    ugr_msgs::ObservationWithCovariance transformed_ob;

    geometry_msgs::PointStamped locStamped;
    locStamped.point = observation.observation.location;
    locStamped.header = this->observations.header;

    try {
      transformed_ob.observation.location =
          this->tfBuffer
              .transform<geometry_msgs::PointStamped>(
                  locStamped, this->base_link_frame,
                  transformed_obs.header.stamp, this->world_frame,
                  ros::Duration(0.05))
              .point;
    } catch (const exception &e) {
      ROS_ERROR("Observation static transform (and perhaps time transform) "
                "failed: %s",
                e.what());
      this->diagPublisher->publishDiagnostic(
          node_fixture::DiagnosticStatusEnum::ERROR, "Observation transform",
          "Static transform (and perhaps time transform) failed!");
      return;
    }

    // Filter out the observation if it is not "in range"
    VectorXf z(2);
    z(0) = pow(transformed_ob.observation.location.x, 2) +
           pow(transformed_ob.observation.location.y, 2);
    z(1) = atan2(transformed_ob.observation.location.y,
                 transformed_ob.observation.location.x);

    if (z(0) > pow(this->max_range, 2) || abs(z(1)) > this->max_half_fov) {
      continue;
    }

    transformed_ob.covariance = observation.covariance;
    transformed_ob.observation.observation_class =
        observation.observation.observation_class;
    transformed_ob.observation.belief = observation.observation.belief;
    transformed_obs.observations.push_back(transformed_ob);
  }

  // --------------------------------------------------------------------
  // ---------------------- Fetch odometry ------------------------------
  // --------------------------------------------------------------------
  // Fetch the current pose estimate (current in: equal to the one of the
  // observations) so that we can estimate dDist and dYaw
  double dX, dY, dYaw = 0; //, dDist;

  geometry_msgs::TransformStamped car_pose;
  try {
    car_pose = this->tfBuffer.lookupTransform(
        this->world_frame, this->base_link_frame, transformed_obs.header.stamp,
        ros::Duration(0.05));
  } catch (const exception &e) {
    ROS_ERROR("car_pose transform failed: %s", e.what());
    this->diagPublisher->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::ERROR, "car_pose transform",
        "failed");
    return;
  }
  const tf2::Quaternion quat(
      car_pose.transform.rotation.x, car_pose.transform.rotation.y,
      car_pose.transform.rotation.z, car_pose.transform.rotation.w);

  double roll, pitch, yaw, x, y;
  x = car_pose.transform.translation.x;
  y = car_pose.transform.translation.y;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  dX = x - this->prev_state[0];
  dY = y - this->prev_state[1];
  dYaw = yaw - this->prev_state[2];

  // Check for reverse
  // double drivingAngle = atan2(y - this->prev_state[1], x -
  // this->prev_state[0]); double angleDifference = abs(drivingAngle - yaw); if
  // (angleDifference > M_PI) {
  //   angleDifference = 2 * M_PI - angleDifference;
  // }

  // bool forward = angleDifference < M_PI_2;
  // dDist = (forward ? 1 : -1) *
  //         pow(pow(x - this->prev_state[0], 2) + pow(y - this->prev_state[1],
  //         2),
  //             0.5);

  this->prev_state = {x, y, yaw};

  // this->diagPublisher->publishDiagnostic(
  //     node_fixture::DiagnosticStatusEnum::OK, "Pose estimate",
  //     "#Dist: " + std::to_string(dDist) + ", yaw: " + std::to_string(dYaw));

  // --------------------------------------------------------------------
  // ---------------------- Add odometry to graph -----------------------
  // --------------------------------------------------------------------

  VertexSE2 *pose = new VertexSE2;
  pose->setId(this->poseIndex);
  pose->setEstimate(SE2(x, y, yaw));
  optimizer.addVertex(pose);

  // add odometry contraint
  if (this->poseIndex > 0) {
    EdgeSE2 *odometry = new EdgeSE2;
    odometry->vertices()[0] = optimizer.vertex(this->poseIndex - 1); // from
    odometry->vertices()[1] = optimizer.vertex(this->poseIndex);     // to
    odometry->setMeasurement(SE2(dX, dY, dYaw));
    odometry->setInformation(
        Matrix3d::Identity()); // ??????????????????????????
    optimizer.addEdge(odometry);
  } else {
    // First pose
    pose->setFixed(true);
  }
  // --------------------------------------------------------------------
  // ---------------------- Add observations to graph -------------------
  // --------------------------------------------------------------------
  for (auto observation : transformed_obs.observations) {
    VertexPointXY *landmark = new VertexPointXY;
    landmark->setId(this->landmarkIndex);
    landmark->setEstimate(Vector2d(x + observation.observation.location.x,
                                   y + observation.observation.location.y));
    optimizer.addVertex(landmark);

    EdgeSE2PointXY *landmarkObservation = new EdgeSE2PointXY;
    landmarkObservation->vertices()[0] =
        optimizer.vertex(this->poseIndex); // pose
    landmarkObservation->vertices()[1] = optimizer.vertex(this->landmarkIndex);
    landmarkObservation->setMeasurement(
        Vector2d(observation.observation.location.x,
                 observation.observation.location.y));

    // Eigen::Matrix3f covarianceMatrix;
    // covarianceMatrix << observation.covariance[0], observation.covariance[1],
    //     observation.covariance[2], observation.covariance[3],
    //     observation.covariance[4], observation.covariance[5],
    //     observation.covariance[6], observation.covariance[7],
    //     observation.covariance[8];
    // landmarkObservation->setInformation(covarianceMatrix.inverse());
    landmarkObservation->setInformation(Matrix2d::Identity());
    optimizer.addEdge(landmarkObservation);

    this->landmarkIndex++;
  }
  this->poseIndex++;

  // --------------------------------------------------------------------
  // ------------------------ Optimization ------------------------------
  // --------------------------------------------------------------------

  // optimizer.setVerbose(true); //The setVerbose(true) function call is used to
  // set the verbosity level of the optimizer object. When verbosity is set to
  // true, the optimizer will output more detailed information about its
  // progress and operations. This can be useful for debugging and understanding
  // how the optimization process is proceeding.

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  // --------------------------------------------------------------------
  // ------------------------ Publish -----------------------------------
  // --------------------------------------------------------------------

  // this->publishOutput(transformed_obs.header.stamp);
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
