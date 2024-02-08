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
#include <visualization_msgs/Marker.h>

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

  this->max_iterations = n.param<int>("max_iterations", 10);
  // Todo: set from parameters
  this->max_range = 15;
  this->max_half_fov = 60 * 0.0174533;
  // this->covariance_pose << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1; // pre-commit
  // doesn't like this
  this->covariance_pose(0, 0) = 0.1;
  this->covariance_pose(0, 1) = 0;
  this->covariance_pose(0, 2) = 0;
  this->covariance_pose(1, 0) = 0;
  this->covariance_pose(1, 1) = 0.1;
  this->covariance_pose(1, 2) = 0;
  this->covariance_pose(2, 0) = 0;
  this->covariance_pose(2, 1) = 0;
  this->covariance_pose(2, 2) = 0.1;

  // Initialize publishers
  this->odomPublisher = n.advertise<nav_msgs::Odometry>("/output/odom", 5);
  this->diagPublisher = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n, "SLAM GrapSLAM"));

  this->landmarkPublisher =
      n.advertise<ugr_msgs::ObservationWithCovarianceArrayStamped>(
          "/graphslam/debug/vertices/landmarks", 0);
  this->posesPublisher = n.advertise<geometry_msgs::PoseArray>(
      "/graphslam/debug/vertices/poses", 0);
  this->edgePosesPublisher = n.advertise<visualization_msgs::Marker>(
      "/graphslam/debug/edges/poses", 0);
  this->edgeLandmarksPublisher = n.advertise<visualization_msgs::Marker>(
      "/graphslam/debug/edges/landmarks", 0);

  // Initialize subscribers
  obs_sub.subscribe(n, "/input/observations", 1);
  tf2_filter.registerCallback(
      boost::bind(&GraphSLAM::handleObservations, this, _1));

  // Initialize variables
  this->gotFirstObservations = false;
  this->prev_state = {0.0, 0.0, 0.0};
  this->vertexCounter = 0;
  this->prevPoseIndex = -1;

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

  this->prev_state = {x, y, yaw};

  // --------------------------------------------------------------------
  // ---------------------- Add odometry to graph -----------------------
  // --------------------------------------------------------------------

  VertexSE2 *newPoseVertex = new VertexSE2;
  newPoseVertex->setId(this->vertexCounter);
  newPoseVertex->setEstimate(SE2(x, y, yaw));
  this->optimizer.addVertex(newPoseVertex);

  // add odometry contraint
  if (this->prevPoseIndex >= 0) {
    EdgeSE2 *odometry = new EdgeSE2;
    odometry->vertices()[0] =
        this->optimizer.vertex(this->prevPoseIndex); // from
    odometry->vertices()[1] = this->optimizer.vertex(this->vertexCounter); // to
    odometry->setMeasurement(SE2(dX, dY, dYaw));
    odometry->setInformation(this->covariance_pose.inverse());

    this->optimizer.addEdge(odometry);
  } else {
    // First pose
    newPoseVertex->setFixed(true);
  }
  this->prevPoseIndex = this->vertexCounter;
  this->vertexCounter++;
  // --------------------------------------------------------------------
  // ---------------------- Add observations to graph -------------------
  // --------------------------------------------------------------------
  for (auto observation : transformed_obs.observations) {
    VertexPointXY *landmark = new VertexPointXY;
    landmark->setId(this->vertexCounter);

    VectorXf obs(2);
    obs << pow(pow(observation.observation.location.x, 2) +
                   pow(observation.observation.location.y, 2),
               0.5),
        atan2(observation.observation.location.y,
              observation.observation.location.x);

    landmark->setEstimate(Vector2d(x + obs(0) * cos(yaw + obs(1)),
                                   y + obs(0) * sin(yaw + obs(1))));
    this->optimizer.addVertex(landmark);

    EdgeSE2PointXY *landmarkObservation = new EdgeSE2PointXY;
    landmarkObservation->vertices()[0] =
        this->optimizer.vertex(this->prevPoseIndex); // pose
    landmarkObservation->vertices()[1] =
        this->optimizer.vertex(this->vertexCounter); // landmark
    landmarkObservation->setMeasurement(
        Vector2d(observation.observation.location.x,
                 observation.observation.location.y));

    Eigen::Matrix2d covarianceMatrix;
    covarianceMatrix << observation.covariance[0], observation.covariance[1],
        observation.covariance[3],
        observation.covariance[4]; // observation gives 3x3 matrix only first 2
                                   // rows and columns are used
    landmarkObservation->setInformation(covarianceMatrix.inverse());
    this->optimizer.addEdge(landmarkObservation);

    this->vertexCounter++;
  }
  // --------------------------------------------------------------------
  // ------------------------ Optimization ------------------------------
  // --------------------------------------------------------------------

  // optimizer.setVerbose(true); //The setVerbose(true) function call is used to
  // set the verbosity level of the optimizer object. When verbosity is set to
  // true, the optimizer will output more detailed information about its
  // progress and operations. This can be useful for debugging and understanding
  // how the optimization process is proceeding.

  // this->optimizer.initializeOptimization();
  // this->optimizer.optimize(this->max_iterations);

  // --------------------------------------------------------------------
  // ------------------------ Publish -----------------------------------
  // --------------------------------------------------------------------

  // this->publishOutput(transformed_obs.header.stamp);

  ugr_msgs::ObservationWithCovarianceArrayStamped landmarks;
  landmarks.header.frame_id = this->map_frame;
  landmarks.header.stamp = transformed_obs.header.stamp;

  geometry_msgs::PoseArray poses;
  poses.header.frame_id = this->map_frame;

  for (const auto &pair :
       this->optimizer.vertices()) { // unordered_map<int, Vertex*>

    VertexSE2 *poseVertex = dynamic_cast<VertexSE2 *>(pair.second);
    if (poseVertex) {
      geometry_msgs::Pose pose;
      pose.position.x = poseVertex->estimate().translation().x();
      pose.position.y = poseVertex->estimate().translation().y();

      tf2::Quaternion q;
      q.setRPY(0, 0, poseVertex->estimate().rotation().angle());

      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();

      poses.poses.push_back(pose);
    }

    VertexPointXY *landmarkVertex = dynamic_cast<VertexPointXY *>(pair.second);
    if (landmarkVertex) {
      ugr_msgs::ObservationWithCovariance map_ob;
      map_ob.observation.location.x = landmarkVertex->estimate().x();
      map_ob.observation.location.y = landmarkVertex->estimate().y();
      map_ob.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
      map_ob.observation.observation_class = 3;
      map_ob.observation.belief = 0;
      landmarks.observations.push_back(map_ob);
    }
  }

  this->landmarkPublisher.publish(landmarks);
  this->posesPublisher.publish(poses);

  // Publish edges
  visualization_msgs::Marker poseEdges;
  poseEdges.header.frame_id = this->map_frame;
  poseEdges.header.stamp = transformed_obs.header.stamp;
  poseEdges.ns = "graphslam";
  poseEdges.type = visualization_msgs::Marker::LINE_LIST;
  poseEdges.action = visualization_msgs::Marker::ADD;
  poseEdges.id = this->prevPoseIndex;
  poseEdges.color.a = 1.0;
  poseEdges.color.r = 1.0;
  poseEdges.color.g = 0.0;
  poseEdges.color.b = 0.0;
  poseEdges.scale.x = 0.1;
  poseEdges.scale.y = 0.1;
  poseEdges.scale.z = 0.1;

  visualization_msgs::Marker landmarkEdges;
  landmarkEdges.header.frame_id = this->map_frame;
  landmarkEdges.header.stamp = transformed_obs.header.stamp;
  landmarkEdges.ns = "graphslam";
  landmarkEdges.type = visualization_msgs::Marker::LINE_LIST;
  landmarkEdges.action = visualization_msgs::Marker::ADD;
  landmarkEdges.id = this->prevPoseIndex;
  landmarkEdges.color.a = 1.0;
  landmarkEdges.color.r = 0.0;
  landmarkEdges.color.g = 1.0;
  landmarkEdges.color.b = 0.0;
  landmarkEdges.scale.x = 0.05;
  landmarkEdges.scale.y = 0.05;
  landmarkEdges.scale.z = 0.05;

  for (const auto &pair : this->optimizer.edges()) {
    pair->vertices()[0]->id();
    EdgeSE2 *edge = dynamic_cast<EdgeSE2 *>(pair);
    if (edge) {
      geometry_msgs::Point p1;
      VertexSE2 *v1 = dynamic_cast<VertexSE2 *>(edge->vertices()[0]);
      p1.x = v1->estimate().translation().x();
      p1.y = v1->estimate().translation().y();
      p1.z = 0.0;

      geometry_msgs::Point p2;
      VertexSE2 *v2 = dynamic_cast<VertexSE2 *>(edge->vertices()[1]);
      p2.x = v2->estimate().translation().x();
      p2.y = v2->estimate().translation().y();
      p2.z = 0.0;

      poseEdges.points.push_back(p1);
      poseEdges.points.push_back(p2);
    }
    EdgeSE2PointXY *edge2 = dynamic_cast<EdgeSE2PointXY *>(pair);
    if (edge2) {
      geometry_msgs::Point p1;
      VertexSE2 *v1 = dynamic_cast<VertexSE2 *>(edge2->vertices()[0]);
      p1.x = v1->estimate().translation().x();
      p1.y = v1->estimate().translation().y();
      p1.z = 0.0;

      geometry_msgs::Point p2;
      VertexPointXY *v2 = dynamic_cast<VertexPointXY *>(edge2->vertices()[1]);
      p2.x = v2->estimate().x();
      p2.y = v2->estimate().y();
      p2.z = 0.0;

      landmarkEdges.points.push_back(p1);
      landmarkEdges.points.push_back(p2);
    }
  }

  this->edgeLandmarksPublisher.publish(landmarkEdges);
  this->edgePosesPublisher.publish(poseEdges);
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
