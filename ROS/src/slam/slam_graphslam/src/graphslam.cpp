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
  this->debug = n.param<bool>("debug", false);

  this->max_iterations = n.param<int>("max_iterations", 10);
  this->association_threshold = n.param<double>("association_threshold", 0.5);
  this->max_range = n.param<double>("max_range", 15);
  this->max_half_angle = n.param<double>("max_half_angle", 60 * 0.0174533);

  // Initialize covariance matrices
  vector<double> cov_pose_vector;
  vector<double> cov_landmark_vector;

  n.param<vector<double>>("covariance_pose", cov_pose_vector,
                          {1, 0, 0, 0, 1, 0, 0, 0, 1});
  n.param<vector<double>>("covariance_landmark", cov_landmark_vector,
                          {0.2, 0, 0, 0.2});

  if (cov_pose_vector.size() != 9)
    throw invalid_argument("The covariance pose must be a vector of size 9");

  if (cov_landmark_vector.size() != 4)
    throw invalid_argument(
        "The covariance landmark must be a vector of size 4");

  Eigen::Matrix3d covariance_pose;
  Eigen::Matrix2d covariance_landmark;

  covariance_pose(0, 0) = cov_pose_vector[0];
  covariance_pose(0, 1) = cov_pose_vector[1];
  covariance_pose(0, 2) = cov_pose_vector[2];
  covariance_pose(1, 0) = cov_pose_vector[3];
  covariance_pose(1, 1) = cov_pose_vector[4];
  covariance_pose(1, 2) = cov_pose_vector[5];
  covariance_pose(2, 0) = cov_pose_vector[6];
  covariance_pose(2, 1) = cov_pose_vector[7];
  covariance_pose(2, 2) = cov_pose_vector[8];

  covariance_landmark(0, 0) = cov_landmark_vector[0];
  covariance_landmark(0, 1) = cov_landmark_vector[1];
  covariance_landmark(1, 0) = cov_landmark_vector[2];
  covariance_landmark(1, 1) = cov_landmark_vector[3];

  this->information_pose = covariance_pose.inverse();
  this->information_landmark = covariance_landmark.inverse();

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

  if (this->debug) {
    // Initialize debug publishers
    this->posesPublisher =
        n.advertise<geometry_msgs::PoseArray>("/output/pose/vertices", 0);
    this->edgePosesPublisher =
        n.advertise<visualization_msgs::Marker>("/output/pose/edges", 0);
    this->edgeLandmarksPublisher =
        n.advertise<visualization_msgs::Marker>("/output/landmark/edges", 0);
  }

  // Initialize subscribers
  obs_sub.subscribe(n, "/input/observations", 1);
  tf2_filter.registerCallback(
      boost::bind(&GraphSLAM::handleObservations, this, _1));

  // Initialize variables
  this->latestTime = 0.0;
  this->gotFirstObservations = false;
  // to keep track of the previous odometry state for calculating the
  // transformation
  this->prev_state = {0.0, 0.0, 0.0};
  // to now wich index the new vertex gets
  this->vertexCounter = 0;
  // to keep track of the previous pose
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
} // doConfigure method

void GraphSLAM::handleObservations(
    const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs) {
  if (!this->isActive()) { // Node lifecycle check
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
} // handleObservations method

void GraphSLAM::step() {
  if (!this->isActive() || !this->gotFirstObservations) {
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

    if (z(0) > pow(this->max_range, 2) || abs(z(1)) > this->max_half_angle) {
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

  double roll, pitch, yaw, x, y;
  x = car_pose.transform.translation.x;
  y = car_pose.transform.translation.y;

  // calculate yaw from quaternion
  const tf2::Quaternion quat(
      car_pose.transform.rotation.x, car_pose.transform.rotation.y,
      car_pose.transform.rotation.z, car_pose.transform.rotation.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  SE2 prev_poseSE2(this->prev_state[0], this->prev_state[1],
                   this->prev_state[2]);
  // set the previous odometry state to the current odometry state
  this->prev_state = {x, y, yaw};
  SE2 poseSE2(x, y, yaw);
  // calculate the transformation from the previous pose to the new pose
  SE2 pose_trans = prev_poseSE2.inverse() * poseSE2;
  // --------------------------------------------------------------------
  // ---------------------- Add odometry to graph -----------------------
  // --------------------------------------------------------------------
  // Add the odometry to the graph
  PoseVertex *newPoseVertex = new PoseVertex;
  newPoseVertex->setId(this->vertexCounter);
  // set the new pose to the current odometry estimate
  newPoseVertex->setEstimate(poseSE2);
  this->optimizer.addVertex(newPoseVertex);

  // add odometry contraint
  if (this->prevPoseIndex >= 0) {
    PoseEdge *odometry = new PoseEdge;
    // the odometry edge is between the previous pose and the new pose
    odometry->vertices()[0] =
        this->optimizer.vertex(this->prevPoseIndex); // from
    odometry->vertices()[1] = this->optimizer.vertex(this->vertexCounter); // to
    // set the odometry measurement to the transformation from the previous pose
    // to the new pose
    odometry->setMeasurement(pose_trans);
    // set the information matrix to the inverse of the covariance matrix
    odometry->setInformation(this->information_pose);
    this->optimizer.addEdge(odometry);
  } else {
    // if this is the first pose, set the pose to fixed
    newPoseVertex->setFixed(true);
  }
  // set the previous pose index to the new pose index
  // this is used to add the odometry edge between the previous pose and the new
  // pose and to add the landmark edges between the new pose and the landmarks
  this->prevPoseIndex = this->vertexCounter;
  this->vertexCounter++;
  // --------------------------------------------------------------------
  // ---------------------- Add observations to graph -------------------
  // --------------------------------------------------------------------
  for (auto observation : transformed_obs.observations) {
    // calculate the location of the landmark in the world frame
    VectorXf obs(2);
    obs << pow(pow(observation.observation.location.x, 2) +
                   pow(observation.observation.location.y, 2),
               0.5),
        atan2(observation.observation.location.y,
              observation.observation.location.x);

    Vector2d loc = Vector2d(x + obs(0) * cos(yaw + obs(1)),
                            y + obs(0) * sin(yaw + obs(1)));

    // check if the landmark already exists
    int associatedLandmarkIndex = -1;
    for (const auto &pair : this->optimizer.vertices()) {
      LandmarkVertex *landmarkVertex =
          dynamic_cast<LandmarkVertex *>(pair.second);
      if (landmarkVertex) {
        Vector2d landmark = landmarkVertex->estimate();
        // if the distance between the landmark and the observation is smaller
        // than the association threshold than the landmark already exists
        if ((loc - landmark).norm() < this->association_threshold) {
          associatedLandmarkIndex = landmarkVertex->id();
          break;
        }
      }
    }
    // if the landmark does not exist, make a new landmark and add it to the
    // graph
    if (associatedLandmarkIndex < 0) {
      LandmarkVertex *landmark = new LandmarkVertex;
      landmark->setId(this->vertexCounter);

      // set the color and belief of the landmark to the observation class
      landmark->setColor(observation.observation.observation_class,
                         observation.observation.belief);

      // set the estimate of the landmark to the location of the landmark in the
      // world frame
      landmark->setEstimate(loc);
      this->optimizer.addVertex(landmark);
      // set the associated landmark index to the new landmark index
      associatedLandmarkIndex = this->vertexCounter;
      this->vertexCounter++;
    }
    // add a constraint between the new pose and the landmark
    LandmarkEdge *landmarkObservation = new LandmarkEdge;
    landmarkObservation->vertices()[0] =
        this->optimizer.vertex(this->prevPoseIndex); // pose
    landmarkObservation->vertices()[1] =
        this->optimizer.vertex(associatedLandmarkIndex); // landmark
    // set the measurement to the transformation from the new pose to the
    // landmark
    landmarkObservation->setMeasurement(poseSE2.inverse() * loc);

    // covarianceMatrix << observation.covariance[0], observation.covariance[1],
    //     observation.covariance[3],
    //     observation.covariance[4]; // observation gives 3x3 matrix only first
    //     2 rows and columns are used
    // landmarkObservation->setInformation(covarianceMatrix.inverse());

    // set the information matrix to the inverse of the covariance matrix
    landmarkObservation->setInformation(this->information_landmark);
    this->optimizer.addEdge(landmarkObservation);
  }

  // --------------------------------------------------------------------
  // ------------------------ Optimization ------------------------------
  // --------------------------------------------------------------------
  // optimize the graph

  // if (this->debug) {
  // this->optimizer.setVerbose(true); //The setVerbose(true) function call is
  //  used to set the verbosity level of the optimizer object. When verbosity is
  //  set to true, the optimizer will output more detailed information about its
  //  progress and operations. This can be useful for debugging and
  //  understanding how the optimization process is proceeding.
  //}

  this->optimizer.initializeOptimization();
  this->optimizer.optimize(this->max_iterations);

  // --------------------------------------------------------------------
  // ------------------------ Kdtree ------------------------------------
  // --------------------------------------------------------------------
  // merge landmarks that are close to each other
  // create a kdtree with the landmarks

  // create a vector of nodes with the landmarks
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

  // if there are more than 2 landmarks, merge the landmarks that are close to
  // each other
  if (nodes.size() > 2) {
    // create a kdtree with the KdNodes
    Kdtree::KdTree tree(&nodes);

    // create a vector of indices of the merged landmarks
    // this is used to keep track of the landmarks that are already merged
    // if a landmark is already merged, it should not be merged again
    vector<int> merged_indices;

    for (auto &node : nodes) {
      // if the node is not already merged
      if (find(merged_indices.begin(), merged_indices.end(), node.index) ==
          merged_indices.end()) {
        // find the nearest neighbors of the node
        Kdtree::KdNodeVector result;
        tree.range_nearest_neighbors(node.point, this->association_threshold,
                                     &result);
        // if there are more than 1 nearest neighbors, merge the landmarks
        if (result.size() > 1) {
          for (auto &neighbor : result) {
            // if the neighbor is not already merged and the index of the
            // neighbor is larger than the index of the node then merge the
            // landmarks the index has to be larger so that the landmarks are
            // always merged in to the smallest index this means less jumping of
            // cones and easier for control progessive pathplaning
            if (neighbor.index > node.index &&
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
  // ------------------------ Publish -----------------------------------
  // --------------------------------------------------------------------
  // publish the odometry and the landmarks
  this->publishOutput(transformed_obs.header.stamp);
} // step method

void GraphSLAM::publishOutput(ros::Time lookupTime) {
  // check that we do not publish faster than the publish rate
  if (std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - this->prev_publish_time)
          .count() < 1.0 / this->publish_rate) {
    return;
  }
  this->prev_publish_time = std::chrono::steady_clock::now();

  // --------------------------------------------------------------------
  // ----------------- Publish odometry ---------------------------------
  // --------------------------------------------------------------------
  // get the pose vertex of the current pose
  PoseVertex *pose_vertex =
      dynamic_cast<PoseVertex *>(this->optimizer.vertex(this->prevPoseIndex));

  // create the odometry message
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
  // create the transform message
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
  // create the observation messages and publish them to the map server
  // global is the landmarks in the world frame
  // local is the landmarks in the base_link frame

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
      global_ob.observation.observation_class = landmarkVertex->getColor();
      local_ob.observation.observation_class = landmarkVertex->getColor();
      global_ob.observation.belief = 0; // mss aantal edges
      local_ob.observation.belief = 0;

      global_ob.observation.index = pair.first;
      local_ob.observation.index = pair.first;

      global_ob.observation.location.x = landmarkVertex->estimate().x();
      global_ob.observation.location.y = landmarkVertex->estimate().y();

      global.observations.push_back(global_ob);

      // landmark to observation
      // calculate the location of the landmark in the base_link frame
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

  // publish the observations to the map server
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
    // if debug is true, publish the poses and edges

    // --------------------------------------------------------------------
    // -------------------- Publish Poses ---------------------------------
    // --------------------------------------------------------------------
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

    // --------------------------------------------------------------------
    // -------------------- Publish edges ---------------------------------
    // --------------------------------------------------------------------
    // create the edge messages and publish them

    // create pose edges message
    visualization_msgs::Marker poseEdges;
    poseEdges.header.frame_id = this->map_frame;
    poseEdges.header.stamp = lookupTime;
    poseEdges.ns = "graphslam";
    poseEdges.type = visualization_msgs::Marker::LINE_LIST;
    // how long the marker will be displayed
    poseEdges.lifetime = ros::Duration(1);
    poseEdges.action = visualization_msgs::Marker::ADD;
    poseEdges.id = this->prevPoseIndex;
    poseEdges.color.a = 1.0; // transparantie
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

    // create landmark edges message
    visualization_msgs::Marker landmarkEdges;
    landmarkEdges.header.frame_id = this->map_frame;
    landmarkEdges.header.stamp = lookupTime;
    landmarkEdges.ns = "graphslam";
    landmarkEdges.type = visualization_msgs::Marker::LINE_LIST;
    landmarkEdges.lifetime = ros::Duration(1);
    landmarkEdges.action = visualization_msgs::Marker::ADD;
    landmarkEdges.id = this->prevPoseIndex;
    landmarkEdges.color.a = 1.0; // transparantie
    landmarkEdges.color.r = 0.0;
    landmarkEdges.color.g = 1.0;
    landmarkEdges.color.b = 0.0;
    landmarkEdges.scale.x = 0.005;
    landmarkEdges.scale.y = 0.005;
    landmarkEdges.scale.z = 0.005;
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
} // publishOutput method
} // namespace slam
