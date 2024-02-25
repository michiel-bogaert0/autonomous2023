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

#include "edge.hpp"
#include "kdtree.hpp"
#include "se2.hpp"
#include "vertex.hpp"

using namespace std;
using namespace Eigen;
using namespace g2o;

namespace slam {

GraphSLAM::GraphSLAM(ros::NodeHandle &n)
    : ManagedNode(n, "graphslam"), n(n), tfListener(tfBuffer),
      base_link_frame(n.param<string>("base_link_frame", "ugr/car_base_link")),
      tf2_filter(obs_sub, tfBuffer, base_link_frame, 1, 0) {
  // this->doConfigure(); // remove this line
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
  this->debug = n.param<bool>("debug", false);

  this->max_iterations = n.param<int>("max_iterations", 10);
  this->association_threshold = n.param<double>("association_threshold", 0.5);
  // Todo: set from launch file
  this->max_range = 15;
  this->max_half_fov = 60 * 0.0174533;

  // Initialize map Service Client
  string SetMap_service =
      n.param<string>("SetMap_service", "/ugr/srv/slam_map_server/set");
  this->setmap_srv_client =
      n.serviceClient<slam_controller::SetMap::Request>(SetMap_service, true);

  this->globalmap_namespace = n.param<string>("globalmap_namespace", "global");
  this->localmap_namespace = n.param<string>("localmap_namespace", "local");

  // Initialize publishers
  this->odomPublisher = n.advertise<nav_msgs::Odometry>("/output/odom", 5);
  this->diagPublisher = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n, "SLAM GrapSLAM"));

  this->landmarkPublisher =
      n.advertise<ugr_msgs::ObservationWithCovarianceArrayStamped>(
          "/output/observations", 0);
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

  // first covariance from odometry
  this->covariance_pose(0, 0) = 0.239653;
  this->covariance_pose(0, 1) = 0.0510531;
  this->covariance_pose(0, 2) = 0.00847513;
  this->covariance_pose(0, 3) = 0.0510531;
  this->covariance_pose(1, 0) = 0.0510531;
  this->covariance_pose(1, 1) = 2.60422;
  this->covariance_pose(1, 2) = 0.261481;
  this->covariance_pose(1, 3) = 0.00847513;
  this->covariance_pose(2, 0) = 0.00847513;
  this->covariance_pose(2, 1) = 0.261481;
  this->covariance_pose(2, 2) = 0.286192;

  // Stole from lidar pkg
  this->covariance_landmark(0, 0) = 0.2;
  this->covariance_landmark(0, 1) = 0;
  this->covariance_landmark(1, 0) = 0;
  this->covariance_landmark(1, 1) = 0.2;

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
    // if (!this->gotFirstObservations) {
    return;
  }

  // --------------------------------------------------------------------
  // ------------------------ Kdtree ------------------------------------
  // --------------------------------------------------------------------
  Kdtree::KdNodeVector nodes;
  for (const auto &pair : this->optimizer.vertices()) {
    LandmarkVertex *landmarkVertex =
        dynamic_cast<LandmarkVertex *>(pair.second);
    if (landmarkVertex) {
      std::vector<double> point(2);
      point[0] = landmarkVertex->estimate().x();
      point[1] = landmarkVertex->estimate().y();
      nodes.push_back(Kdtree::KdNode(point, NULL, pair.first));
    }
  }
  if (nodes.size() > 2) {
    Kdtree::KdTree tree(&nodes);

    vector<int> merged_indices;

    for (auto &node : nodes) {
      if (find(merged_indices.begin(), merged_indices.end(), node.index) ==
          merged_indices.end()) {
        // if not already merged
        Kdtree::KdNodeVector result;
        tree.range_nearest_neighbors(node.point, this->association_threshold,
                                     &result);
        if (result.size() > 1) {
          for (auto &neighbor : result) {
            if (neighbor.index != node.index &&
                find(merged_indices.begin(), merged_indices.end(),
                     neighbor.index) == merged_indices.end()) {
              this->optimizer.mergeVertices(
                  this->optimizer.vertex(node.index),
                  this->optimizer.vertex(neighbor.index), true);
              merged_indices.push_back(neighbor.index);
            }
          }
        }
      }
    }
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
    if (observation.observation.observation_class == 2) { // verwijderen
      continue;
    }
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

  SE2 prev_poseSE2(this->prev_state[0], this->prev_state[1],
                   this->prev_state[2]);
  this->prev_state = {x, y, yaw};
  SE2 poseSE2(x, y, yaw);
  SE2 pose_trans = prev_poseSE2.inverse() * poseSE2;
  // --------------------------------------------------------------------
  // ---------------------- Add odometry to graph -----------------------
  // --------------------------------------------------------------------
  // Add the odometry to the graph
  PoseVertex *newPoseVertex = new PoseVertex;
  newPoseVertex->setId(this->vertexCounter);
  newPoseVertex->setEstimate(poseSE2);
  this->optimizer.addVertex(newPoseVertex);

  // add odometry contraint
  if (this->prevPoseIndex >= 0) {
    PoseEdge *odometry = new PoseEdge;
    odometry->vertices()[0] =
        this->optimizer.vertex(this->prevPoseIndex); // from
    odometry->vertices()[1] = this->optimizer.vertex(this->vertexCounter); // to
    odometry->setMeasurement(pose_trans);
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
    VectorXf obs(2);
    obs << pow(pow(observation.observation.location.x, 2) +
                   pow(observation.observation.location.y, 2),
               0.5),
        atan2(observation.observation.location.y,
              observation.observation.location.x);

    Vector2d loc = Vector2d(x + obs(0) * cos(yaw + obs(1)),
                            y + obs(0) * sin(yaw + obs(1)));

    int associatedLandmarkIndex = -1;
    for (const auto &pair : this->optimizer.vertices()) {
      LandmarkVertex *landmarkVertex =
          dynamic_cast<LandmarkVertex *>(pair.second);
      if (landmarkVertex) {
        Vector2d landmark = landmarkVertex->estimate();
        if ((loc - landmark).norm() < this->association_threshold) {
          // landmark already exists
          associatedLandmarkIndex = landmarkVertex->id();
          break;
        }
      }
    }
    if (associatedLandmarkIndex < 0) {
      // landmark does not exist
      // make new landmark and add to graph
      LandmarkVertex *landmark = new LandmarkVertex;
      landmark->setId(this->vertexCounter);
      landmark->setColor(observation.observation.observation_class);

      landmark->setEstimate(loc);
      this->optimizer.addVertex(landmark);
      associatedLandmarkIndex = this->vertexCounter;
      this->vertexCounter++;
    }

    LandmarkEdge *landmarkObservation = new LandmarkEdge;
    landmarkObservation->vertices()[0] =
        this->optimizer.vertex(this->prevPoseIndex); // pose
    landmarkObservation->vertices()[1] =
        this->optimizer.vertex(associatedLandmarkIndex); // landmark
    landmarkObservation->setMeasurement(poseSE2.inverse() * loc);

    // covarianceMatrix << observation.covariance[0], observation.covariance[1],
    //     observation.covariance[3],
    //     observation.covariance[4]; // observation gives 3x3 matrix only first
    //     2 rows and columns are used
    // landmarkObservation->setInformation(covarianceMatrix.inverse());

    // std::ostringstream matrixStream;
    // matrixStream << covarianceMatrix;
    // ROS_INFO("Covariance: \n%s", matrixStream.str().c_str());

    landmarkObservation->setInformation(this->covariance_landmark.inverse());
    this->optimizer.addEdge(landmarkObservation);
  }

  // --------------------------------------------------------------------
  // ------------------------ Optimization ------------------------------
  // --------------------------------------------------------------------

  // this->optimizer.setVerbose(true); //The setVerbose(true) function call is
  // used to set the verbosity level of the optimizer object. When verbosity is
  // set to true, the optimizer will output more detailed information about its
  // progress and operations. This can be useful for debugging and understanding
  // how the optimization process is proceeding.

  this->optimizer.initializeOptimization();
  this->optimizer.optimize(this->max_iterations);

  // --------------------------------------------------------------------
  // ------------------------ Publish -----------------------------------
  // --------------------------------------------------------------------
  this->publishOutput(transformed_obs.header.stamp);
}

void GraphSLAM::publishOutput(ros::Time lookupTime) {

  if (std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - this->prev_publish_time)
          .count() < 1.0 / this->publish_rate) {
    return;
  }

  this->prev_publish_time = std::chrono::steady_clock::now();

  // --------------------------------------------------------------------
  // ----------------- Publish odometry ---------------------------------
  // --------------------------------------------------------------------

  PoseVertex *pose_vertex =
      dynamic_cast<PoseVertex *>(this->optimizer.vertex(this->prevPoseIndex));

  nav_msgs::Odometry odom;

  odom.header.stamp = lookupTime;
  odom.header.frame_id = this->map_frame;
  odom.child_frame_id = this->slam_base_link_frame;

  odom.pose.pose.position.x = pose_vertex->estimate().translation().x();
  odom.pose.pose.position.y = pose_vertex->estimate().translation().y();

  tf2::Quaternion quat;
  quat.setRPY(0, 0, pose_vertex->estimate().rotation().angle());

  odom.pose.pose.orientation.x = quat.getX();
  odom.pose.pose.orientation.y = quat.getY();
  odom.pose.pose.orientation.z = quat.getZ();
  odom.pose.pose.orientation.w = quat.getW();

  this->odomPublisher.publish(odom);

  // --------------------------------------------------------------------
  // ----------------- TF Transformation --------------------------------
  // --------------------------------------------------------------------
  tf2::Transform transform(
      quat, tf2::Vector3(pose_vertex->estimate().translation().x(),
                         pose_vertex->estimate().translation().y(), 0));

  geometry_msgs::TransformStamped transformMsg;
  transformMsg.header.frame_id = this->map_frame;
  transformMsg.header.stamp = lookupTime;
  transformMsg.child_frame_id = this->slam_base_link_frame;

  transformMsg.transform.translation.x =
      pose_vertex->estimate().translation().x();
  transformMsg.transform.translation.y =
      pose_vertex->estimate().translation().y();

  transformMsg.transform.rotation.x = quat.getX();
  transformMsg.transform.rotation.y = quat.getY();
  transformMsg.transform.rotation.z = quat.getZ();
  transformMsg.transform.rotation.w = quat.getW();

  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transformMsg);

  // --------------------------------------------------------------------
  // -------------- Publish Landmarks to map server ---------------------
  // --------------------------------------------------------------------

  // Create the observation_msgs things
  ugr_msgs::ObservationWithCovarianceArrayStamped global;
  ugr_msgs::ObservationWithCovarianceArrayStamped local;
  global.header.frame_id = this->map_frame;
  global.header.stamp = lookupTime;
  local.header.frame_id = this->slam_base_link_frame;
  local.header.stamp = lookupTime;

  for (const auto &pair :
       this->optimizer.vertices()) { // unordered_map<int, Vertex*>

    LandmarkVertex *landmarkVertex =
        dynamic_cast<LandmarkVertex *>(pair.second);
    if (landmarkVertex) {

      ugr_msgs::ObservationWithCovariance global_ob;
      ugr_msgs::ObservationWithCovariance local_ob;

      global_ob.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
      local_ob.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
      global_ob.observation.observation_class = landmarkVertex->color;
      local_ob.observation.observation_class = landmarkVertex->color;
      global_ob.observation.belief = 0;
      local_ob.observation.belief = 0;

      global_ob.observation.location.x = landmarkVertex->estimate().x();
      global_ob.observation.location.y = landmarkVertex->estimate().y();

      global.observations.push_back(global_ob);

      // landmark to observation
      VectorXf obs(2);
      float dx = landmarkVertex->estimate().x() -
                 pose_vertex->estimate().translation().x();
      float dy = landmarkVertex->estimate().y() -
                 pose_vertex->estimate().translation().y();

      obs(0) = pow(pow(dx, 2) + pow(dy, 2), 0.5);
      obs(1) = atan2(dy, dx) - pose_vertex->estimate().rotation().angle();

      local_ob.observation.location.x = obs(0) * cos(obs(1));
      local_ob.observation.location.y = obs(0) * sin(obs(1));
      local.observations.push_back(local_ob);
    }
  }

  if (this->setmap_srv_client.exists()) {
    this->diagPublisher->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::OK, "SetMap service call",
        "exists");
    // Initialize global request
    slam_controller::SetMap global_srv;
    global_srv.request.map = global;
    global_srv.request.name = this->globalmap_namespace;

    // Set global map with Service
    if (!this->setmap_srv_client.call(global_srv)) {
      ROS_WARN("Global map service call failed!");
      this->diagPublisher->publishDiagnostic(
          node_fixture::DiagnosticStatusEnum::WARN, "Global map service call",
          "failed");
    } else {
      this->diagPublisher->publishDiagnostic(
          node_fixture::DiagnosticStatusEnum::OK, "Global map service call",
          "success");
    }

    // Initialize local request
    slam_controller::SetMap local_srv;
    local_srv.request.map = local;
    local_srv.request.name = this->localmap_namespace;

    // Set local map with Service
    if (!this->setmap_srv_client.call(local_srv)) {
      ROS_WARN("Local map service call failed!");
      this->diagPublisher->publishDiagnostic(
          node_fixture::DiagnosticStatusEnum::WARN, "Local map service call",
          "failed");
    } else {
      this->diagPublisher->publishDiagnostic(
          node_fixture::DiagnosticStatusEnum::OK, "Local map service call",
          "success");
    }
  } else {
    ROS_WARN("SetMap service call does not exist (yet)!");
    this->diagPublisher->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::WARN, "SetMap service call",
        "Does not exist!");
  }

  // --------------------------------------------------------------------
  // --------------------- publish debug --------------------------------
  // --------------------------------------------------------------------
  if (this->debug) {

    // --------------------------------------------------------------------
    // ----------------- Publish Landmarks --------------------------------
    // --------------------------------------------------------------------

    ugr_msgs::ObservationWithCovarianceArrayStamped landmarks;
    landmarks.header.frame_id = this->map_frame;
    landmarks.header.stamp = lookupTime;

    for (const auto &pair :
         this->optimizer.vertices()) { // unordered_map<int, Vertex*>

      LandmarkVertex *landmarkVertex =
          dynamic_cast<LandmarkVertex *>(pair.second);
      if (landmarkVertex) {
        ugr_msgs::ObservationWithCovariance map_ob;
        map_ob.observation.location.x = landmarkVertex->estimate().x();
        map_ob.observation.location.y = landmarkVertex->estimate().y();
        map_ob.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        map_ob.observation.observation_class = landmarkVertex->color;
        map_ob.observation.belief = 0;
        landmarks.observations.push_back(map_ob);
      }
    }

    this->landmarkPublisher.publish(landmarks);
    // Publish odometry poses
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = this->map_frame;

    for (const auto &pair :
         this->optimizer.vertices()) { // unordered_map<int, Vertex*>

      PoseVertex *poseVertex = dynamic_cast<PoseVertex *>(pair.second);
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
    }
    this->posesPublisher.publish(poses);

    // Publish edges
    visualization_msgs::Marker poseEdges;
    poseEdges.header.frame_id = this->map_frame;
    poseEdges.header.stamp = lookupTime;
    poseEdges.ns = "graphslam";
    poseEdges.type = visualization_msgs::Marker::LINE_LIST;
    poseEdges.lifetime = ros::Duration(1);
    poseEdges.action = visualization_msgs::Marker::ADD;
    poseEdges.id = this->prevPoseIndex;
    poseEdges.color.a = 1.0;
    poseEdges.color.r = 1.0;
    poseEdges.color.g = 0.0;
    poseEdges.color.b = 0.0;
    poseEdges.scale.x = 0.1;
    poseEdges.scale.y = 0.1;
    poseEdges.scale.z = 0.1;
    poseEdges.pose.orientation.x = 0.0;
    poseEdges.pose.orientation.y = 0.0;
    poseEdges.pose.orientation.z = 0.0;
    poseEdges.pose.orientation.w = 1.0;

    visualization_msgs::Marker landmarkEdges;
    landmarkEdges.header.frame_id = this->map_frame;
    landmarkEdges.header.stamp = lookupTime;
    landmarkEdges.ns = "graphslam";
    landmarkEdges.type = visualization_msgs::Marker::LINE_LIST;
    landmarkEdges.lifetime = ros::Duration(1);
    landmarkEdges.action = visualization_msgs::Marker::ADD;
    landmarkEdges.id = this->prevPoseIndex;
    landmarkEdges.color.a = 1.0;
    landmarkEdges.color.r = 0.0;
    landmarkEdges.color.g = 1.0;
    landmarkEdges.color.b = 0.0;
    landmarkEdges.scale.x = 0.05;
    landmarkEdges.scale.y = 0.05;
    landmarkEdges.scale.z = 0.05;
    landmarkEdges.pose.orientation.x = 0.0;
    landmarkEdges.pose.orientation.y = 0.0;
    landmarkEdges.pose.orientation.z = 0.0;
    landmarkEdges.pose.orientation.w = 1.0;

    bool isPoseEdge = false;

    for (const auto &pair : this->optimizer.edges()) {
      pair->vertices()[0]->id();
      PoseEdge *edge = dynamic_cast<PoseEdge *>(pair);
      if (edge) {
        isPoseEdge = true;
        geometry_msgs::Point p1;
        PoseVertex *v1 = dynamic_cast<PoseVertex *>(edge->vertices()[0]);
        p1.x = v1->estimate().translation().x();
        p1.y = v1->estimate().translation().y();
        p1.z = 0.0;

        geometry_msgs::Point p2;
        PoseVertex *v2 = dynamic_cast<PoseVertex *>(edge->vertices()[1]);
        p2.x = v2->estimate().translation().x();
        p2.y = v2->estimate().translation().y();
        p2.z = 0.0;

        poseEdges.points.push_back(p1);
        poseEdges.points.push_back(p2);
      }
      LandmarkEdge *edge2 = dynamic_cast<LandmarkEdge *>(pair);
      if (edge2) {
        geometry_msgs::Point p1;
        PoseVertex *v1 = dynamic_cast<PoseVertex *>(edge2->vertices()[0]);
        p1.x = v1->estimate().translation().x();
        p1.y = v1->estimate().translation().y();
        p1.z = 0.0;

        geometry_msgs::Point p2;
        LandmarkVertex *v2 =
            dynamic_cast<LandmarkVertex *>(edge2->vertices()[1]);
        p2.x = v2->estimate().x();
        p2.y = v2->estimate().y();
        p2.z = 0.0;

        landmarkEdges.points.push_back(p1);
        landmarkEdges.points.push_back(p2);
      }
    }

    this->edgeLandmarksPublisher.publish(landmarkEdges);
    if (isPoseEdge) {
      this->edgePosesPublisher.publish(poseEdges);
    }
  }
}
} // namespace slam
