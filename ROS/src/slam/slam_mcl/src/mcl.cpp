// #define _GLIBCXX_USE_CXX11_ABI 0
#include "mcl.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "particle.hpp"
#include "tf_service/buffer_client.h"

#include <Eigen/Dense>

#include "ros/console.h"
#include "ros/ros.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

#include "message_filters/subscriber.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include "dbscan.hpp"
#include "helpers.hpp"
#include "kdtree.hpp"
#include "kdtreepoint.hpp"

#include <nav_msgs/Odometry.h>

#include <string>

#include <ugr_msgs/ObservationWithCovariance.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

#include <cmath>

using namespace std;
using namespace Eigen;

namespace slam {

MCL::MCL(ros::NodeHandle &n)
    : ManagedNode(n, "slam_mcl"), n(n), tfListener(tfBuffer),
      base_link_frame(n.param<string>("base_link_frame", "ugr/car_base_link")),
      tf2_filter(obs_sub, tfBuffer, base_link_frame, 1, 0) {}

void MCL::doConfigure() {
  this->slam_base_link_frame =
      this->n.param<string>("slam_base_link_frame", "ugr/slam_base_link");
  this->world_frame = this->n.param<string>("world_frame", "ugr/car_odom");
  this->map_frame = this->n.param<string>("map_frame", "ugr/map");
  this->particle_count = this->n.param<int>("particle_count", 100);
  this->effective_particle_count =
      this->n.param<int>("effective_particle_count", 75);
  this->eps = this->n.param<double>("eps", 2.0);
  this->max_range = this->n.param<double>("max_range", 15);
  this->max_half_fov = this->n.param<double>("max_half_angle", 60 * 0.0174533);
  this->observe_dt = this->n.param<double>("observe_dt", 0.2);
  this->prev_state = {0, 0, 0};
  this->Q = Eigen::MatrixXf(2, 2);
  this->R = Eigen::MatrixXf(2, 2);
  this->yaw_unwrap_threshold =
      this->n.param<float>("yaw_unwrap_threshold", M_PI * 1.3);

  this->odomPublisher = n.advertise<nav_msgs::Odometry>("/output/odom", 5);
  this->diagPublisher = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n, "SLAM MCL"));

  obs_sub.subscribe(n, "/input/observations", 1);
  tf2_filter.registerCallback(boost::bind(&MCL::handleObservations, this, _1));

  // "/input/map" --> changed to param to change topic between missions with
  // node lifecylce
  mapSubscriber = n.subscribe(
      this->n.param<string>("path_to_map", "/ugr/car/map/slam/global"), 1,
      &MCL::handleMap, this);

  vector<double> QAsVector;
  vector<double> RAsVector;

  n.param<vector<double>>("input_noise", QAsVector, {0.1, 0.0, 0.0, 0.02});
  n.param<vector<double>>("measurement_covariance", RAsVector,
                          {0.3, 0.0, 0.0, 0.05});

  if (QAsVector.size() != 4)
    throw invalid_argument(
        "Q (measurement_covariance) Must be a vector of size 4");

  if (RAsVector.size() != 4)
    throw invalid_argument("R (input_noise) Must be a vector of size 4");

  this->Q(0, 0) = pow(QAsVector[0], 2);
  this->Q(0, 1) = pow(QAsVector[1], 2);
  this->Q(1, 0) = pow(QAsVector[2], 2);
  this->Q(1, 1) = pow(QAsVector[3], 2);

  this->R(0, 0) = pow(RAsVector[0], 2);
  this->R(0, 1) = pow(RAsVector[1], 2);
  this->R(1, 0) = pow(RAsVector[2], 2);
  this->R(1, 1) = pow(RAsVector[3], 2);

  // Try to fetch the initial position of the car using tf service
  tf_service::BufferClient buffer("/tf_service");
  buffer.waitForServer();
  // Use it like any other TF2 buffer.
  std::string errstr;

  double stand_dev = n.param<double>("setup_stand_dev", 0.1);
  this->particles.clear();
  for (int i = 0; i < this->particle_count; i++) {
    this->particles.push_back(Particle(stand_dev));
  }

  // For particle initial position
  if (buffer.canTransform(this->map_frame, this->slam_base_link_frame,
                          ros::Time(0), ros::Duration(1), &errstr)) {
    geometry_msgs::TransformStamped initial_particle_pose =
        buffer.lookupTransform(this->map_frame, this->slam_base_link_frame,
                               ros::Time(0), ros::Duration(1));

    const tf2::Quaternion quat(initial_particle_pose.transform.rotation.x,
                               initial_particle_pose.transform.rotation.y,
                               initial_particle_pose.transform.rotation.z,
                               initial_particle_pose.transform.rotation.w);

    for (int i = 0; i < this->particles.size(); i++) {
      Particle &particle = this->particles[i];

      VectorXf xv = particle.pose();
      xv(0) += initial_particle_pose.transform.translation.x;
      xv(1) += initial_particle_pose.transform.translation.y;

      double roll, pitch, yaw;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      xv(2) = yaw;

      particle.setPose(xv);
    }

    // For prev_state
    if (buffer.canTransform(this->world_frame, this->base_link_frame,
                            initial_particle_pose.header.stamp,
                            ros::Duration(1), &errstr)) {
      geometry_msgs::TransformStamped initial_car_pose = buffer.lookupTransform(
          this->world_frame, this->base_link_frame,
          initial_particle_pose.header.stamp, ros::Duration(1));

      const tf2::Quaternion prev_quat(initial_car_pose.transform.rotation.x,
                                      initial_car_pose.transform.rotation.y,
                                      initial_car_pose.transform.rotation.z,
                                      initial_car_pose.transform.rotation.w);

      double roll, pitch, yaw;
      this->prev_state[0] = initial_car_pose.transform.translation.x;
      this->prev_state[1] = initial_car_pose.transform.translation.y;
      tf2::Matrix3x3(prev_quat).getRPY(roll, pitch, yaw);
      this->prev_state[2] = yaw;
    }
  }

  ROS_INFO("MCL has started!");
  diagPublisher->publishDiagnostic(node_fixture::DiagnosticStatusEnum::OK,
                                   "Status", "Started.");
}

/**
 * Subscriber handler for a "map". Takes the incoming map and prepares the
 * system, such as the k-d trees
 *
 * Args:
 *  obs: a reference to the received ObservationWithCovarianceArrayStamped
 * message containing the map
 */
void MCL::handleMap(
    const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs) {
  if (!this->isActive()) {
    return;
  }

  vector<KDTreePoint> points[LANDMARK_CLASS_COUNT];

  // Convert the message to the vector<Landmark> thing
  _map.clear();
  int id = 0;
  for (auto observation : obs->observations) {
    Landmark landmark;
    VectorXf pose(2);
    MatrixXf variance(3, 3);

    // landmark.landmarkClass = observation.observation.observation_class; //
    // linting (because it is not used)
    pose(0) = observation.observation.location.x;
    pose(1) = observation.observation.location.y;

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        variance(i, j) = observation.covariance[i * 3 + j];
      }
    }

    landmark.pose = pose;
    landmark.variance = variance;

    _map.push_back(landmark);

    points[observation.observation.observation_class].push_back(
        KDTreePoint(pose, id));
    id++;
  }

  // Build KDTrees
  for (int i = 0; i < LANDMARK_CLASS_COUNT; i++) {
    _trees[i].build(points[i]);
  }

  ROS_INFO("Took in new map!");
}

/**
 * Applies the motion update to a given particle
 *
 * Args:
 *  Particle: the particle to move
 *  dDist: relative distance moved (euclidian)
 *  dYaw: relative yaw change
 */
void MCL::motion_update(Particle &particle, double dDist, double dYaw) {

  // Add noise
  VectorXf A(2);
  A(0) = dDist;
  A(1) = dYaw;
  VectorXf VG(2);
  VG = multivariate_gauss(A, this->Q, 1);
  dDist = VG(0);
  dYaw = VG(1);

  // predict state
  VectorXf xv = particle.pose();
  VectorXf xv_temp(3);
  xv_temp << xv(0) + dDist * cos(dYaw + xv(2)),
      xv(1) + dDist * sin(dYaw + xv(2)), pi_to_pi2(xv(2) + dYaw);
  particle.setPose(xv_temp);
}

/**
 * Converts landmark(s) to observation(s) and back.
 *
 * An observation is relative to the sensor in polar coordinates (range,
 * bearing) A landmark is absolute to the world frame in cartesian coordinates
 * (x, y)
 *
 * Argument signature is always (input, output, robot pose)
 */
void MCL::landmarks_to_observations(vector<VectorXf> &lm, vector<VectorXf> &obs,
                                    VectorXf &pose) {
  for (int i = 0; i < lm.size(); i++) {
    VectorXf observation(2);
    this->landmark_to_observation(lm[i], observation, pose);
    obs.push_back(observation);
  }
}
void MCL::landmark_to_observation(VectorXf &lm, VectorXf &obs, VectorXf &pose) {
  float dx = lm(0) - pose(0);
  float dy = lm(1) - pose(1);

  obs(0) = pow(pow(dx, 2) + pow(dy, 2), 0.5);
  obs(1) = atan2(dy, dx) - pose(2);
}
void MCL::observations_to_landmarks(vector<VectorXf> &obs, vector<VectorXf> &lm,
                                    VectorXf &pose) {
  for (int i = 0; i < lm.size(); i++) {
    VectorXf landmark(2);
    this->observation_to_landmark(obs[i], landmark, pose);
    lm.push_back(landmark);
  }
}
void MCL::observation_to_landmark(VectorXf &obs, VectorXf &lm, VectorXf &pose) {
  lm(0) = pose(0) + obs(0) * cos(pose(2) + obs(1));
  lm(1) = pose(1) + obs(0) * sin(pose(2) + obs(1));
}

/**
 * Build associations between observation messages and the currently set map.
 *
 * Args:
 *  particle[in]: the particle to build associations for
 *  observations[in]: the raw messages (so relative to base link) that need to
 * be associated landmarks[out]: the landmark means (coming from observations)
 * that were actually 'associated' indices[out]: the indices of landmarks on the
 * current map that were 'associated'
 */
void MCL::build_associations(
    Particle &particle,
    ugr_msgs::ObservationWithCovarianceArrayStamped &observations,
    vector<VectorXf> &landmarks, vector<int> &indices) {
  landmarks.clear();

  if (_map.size() == 0) {
    diagPublisher->publishDiagnostic(node_fixture::DiagnosticStatusEnum::WARN,
                                     "Map size", "0");
    return;
  }

  diagPublisher->publishDiagnostic(node_fixture::DiagnosticStatusEnum::OK,
                                   "Map size", std::to_string(_map.size()));

  for (auto observation : observations.observations) {

    if (observation.observation.observation_class < 0 ||
        observation.observation.observation_class > LANDMARK_CLASS_COUNT)
      continue;

    // Prepare observation
    VectorXf obsAsVector(2);
    obsAsVector << pow(pow(observation.observation.location.x, 2) +
                           pow(observation.observation.location.y, 2),
                       0.5),
        atan2(observation.observation.location.y,
              observation.observation.location.x);

    VectorXf landmark(2);
    this->observation_to_landmark(obsAsVector, landmark, particle.pose());

    const KDTreePoint query(landmark, -1);

    double distance;
    int kdpointIndex =
        _trees[observation.observation.observation_class].nnSearch(query,
                                                                   &distance);

    if (distance < this->eps) {
      KDTreePoint point = (_trees[observation.observation.observation_class]
                               .getPoints())[kdpointIndex];
      indices.push_back(point.getId());
      landmarks.push_back(obsAsVector);
    }
  }
}

/**
 * Resamples the particle set. The odds of an old particle being 'chosen' to be
 * part of the new set is proportionate with its weights. An old particle can be
 * 'chosen' multiple times, but there will be at least effective_particle_count
 * 'unique' particles. When an old particle gets chosen, its map and everything
 * gets copied into the new set.
 */
void MCL::resample_particles() {
  unsigned long i;
  unsigned long N = particles.size();
  VectorXf w(N);

  for (i = 0; i < N; i++) {
    w(i) = particles[i].w();
  }

  float ws = w.sum();
  for (i = 0; i < N; i++) {
    particles[i].setW(w(i) / ws);
  }

  float Neff = 0.0;
  vector<int> keep;

  stratified_resample(w, keep, Neff);

  vector<Particle> old_particles = vector<Particle>(particles);
  particles.resize(keep.size());

  if ((Neff < effective_particle_count)) {
    for (i = 0; i < keep.size(); i++) {
      particles[i] = old_particles[keep[i]];
    }

    for (i = 0; i < N; i++) {
      float new_w = 1.0f / (float)N;
      particles[i].setW(new_w);
    }
  }
}

/**
 * Applies the sensor measurement model to the particle
 *
 * Args:
 *  particle: the paricle to use
 *  z: the (range, bearing) measurements
 *  associations: the association data (with which map landmarks does z
 * correspond?)
 *
 * Returns:
 *  the likelihood of these measurements, given the current state of the
 * particle
 */
double MCL::sensor_update(Particle &particle, vector<VectorXf> &z,
                          vector<int> &associations) {

  // First convert map to observations, based on particle
  vector<VectorXf> mapLandmarks;
  for (int association : associations) {
    mapLandmarks.push_back(_map[association].pose);
  }

  vector<VectorXf> expectedObservations;
  this->landmarks_to_observations(mapLandmarks, expectedObservations,
                                  particle.pose());

  // Calculate weight
  double weight = 1.0;

  for (unsigned int i = 0; i < associations.size(); i++) {
    MatrixXf sigma = _map[associations[i]].variance;
    VectorXf difference = z[i] - expectedObservations[i];

    difference[1] = pi_to_pi(difference[1]);
    if (sigma.determinant() < 0.001)
      sigma = this->R;

    float den = 2 * M_PI * sqrt(sigma.determinant());
    float num =
        std::exp(-0.5 * difference.transpose() * sigma.inverse() * difference);
    weight *= 1.0 / sqrt(pow(2 * M_PI, 2) * sigma.determinant()) *
              exp(-0.5 * difference.transpose() * sigma.inverse() * difference);
  }

  return weight;
}

/**
 * Handles incoming observations from perception
 *
 * Args:
 *  obs: the observations themselves
 */
void MCL::handleObservations(
    const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs) {
  if (!this->isActive()) {
    return;
  }

  // Backwards time detection
  if (this->latestTime - obs->header.stamp.toSec() > 0.5 &&
      this->latestTime > 0.0) {
    // Reset
    ROS_WARN("Time went backwards! Resetting MCL...");
    this->diagPublisher->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::WARN, "Backwards time",
        "MCL reset (time went backwards)!");

    this->particles.clear();
    for (int i = 0; i < this->particle_count; i++) {
      this->particles.push_back(Particle());
    }

    this->prev_state = {0.0, 0.0, 0.0};
    this->latestTime = 0.0;

    return;
  } else {
    this->latestTime = obs->header.stamp.toSec();
  }

  this->observations = *obs;
}

void MCL::active() {

  // Should the particle filter ignore the observations or not?
  chrono::steady_clock::time_point time = chrono::steady_clock::now();
  bool doSensorUpdate =
      abs(std::chrono::duration_cast<std::chrono::duration<double>>(
              time - this->prev_time)
              .count()) > this->observe_dt;

  if (doSensorUpdate)
    this->prev_time = time;

  // Transform observations into the car_base_link frame to current time and
  // apply a filter
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
                  ros::Duration(0.1))
              .point;
    } catch (const std::exception &e) {
      ROS_ERROR("observation transform failed: %s", e.what());
      return;
    }

    // Filter out the observation if it is not "in range". Increases performance
    VectorXf z(2);
    z(0) = pow(pow(transformed_ob.observation.location.x, 2) +
                   pow(transformed_ob.observation.location.y, 2),
               0.5);
    z(1) = atan2(transformed_ob.observation.location.y,
                 transformed_ob.observation.location.x);

    if (z(0) > this->max_range || abs(z(1)) > this->max_half_fov) {
      continue;
    }

    if (observation.observation.observation_class < 0 ||
        observation.observation.observation_class > LANDMARK_CLASS_COUNT)
      continue;

    transformed_ob.covariance = observation.covariance;
    transformed_ob.observation.observation_class =
        observation.observation.observation_class;
    transformed_obs.observations.push_back(transformed_ob);
  }

  // Fetch the current pose estimate so that we can estimate dDist and dYaw
  double dDist, dYaw = 0;

  geometry_msgs::TransformStamped car_pose;
  try {
    car_pose = this->tfBuffer.lookupTransform(
        this->world_frame, this->base_link_frame, transformed_obs.header.stamp,
        ros::Duration(0.1));
    this->diagPublisher->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::OK, "car_pose transform",
        "Transform success!");
  } catch (const std::exception &e) {
    ROS_ERROR("car_pose transform failed: %s", e.what());
    this->diagPublisher->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::ERROR, "car_pose transform",
        "Transform failed!");
    return;
  }
  const tf2::Quaternion quat(
      car_pose.transform.rotation.x, car_pose.transform.rotation.y,
      car_pose.transform.rotation.z, car_pose.transform.rotation.w);

  double roll, pitch, yaw, x, y;
  x = car_pose.transform.translation.x;
  y = car_pose.transform.translation.y;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  dYaw = yaw - this->prev_state[2];
  dDist = pow(pow(x - this->prev_state[0], 2) + pow(y - this->prev_state[1], 2),
              0.5);

  for (int j = 0; j < particles.size(); j++) {
    Particle &particle = particles[j];
    this->motion_update(particle, dDist, dYaw);
  }

  // Step 2: sensor update (if allowed)
  if (doSensorUpdate) {

    for (int j = 0; j < particles.size(); j++) {

      Particle &particle = particles[j];

      // Get landmark associations (per particle!)
      // 'observations' are the observations that were actually matched
      vector<int> indices;
      vector<VectorXf> observations;
      this->build_associations(particle, transformed_obs, observations,
                               indices);

      // Sensor update
      particle.setW(this->sensor_update(particle, observations, indices));
    }

    // Resampling step
    resample_particles();
  }

  // Finalizing
  this->prev_state = {x, y, yaw};

  // Done ! Now produce the output
  this->publishOutput(transformed_obs.header.stamp);

  this->observations.observations.clear();
}

void MCL::publishOutput(ros::Time lookupTime) {

  // Calculate statistical mean and covariance of the particle filter (pose)
  // Also gather some other information while we are looping

  vector<float> poseX;
  vector<float> poseY;
  vector<float> poseYaw;

  float x = 0.0;
  float y = 0.0;
  float yaw = 0.0;
  float maxW = -10000.0;
  // float totalW = 0.0; linting
  for (int i = 0; i < this->particles.size(); i++) {

    Particle &particle = this->particles[i];

    float w = particle.w();

    if (isnan(w) || w < 0.001) {
      w = 0.001;
    }

    poseX.push_back(particle.pose()(0));
    poseY.push_back(particle.pose()(1));
    poseYaw.push_back(particle.pose()(2));

    float curYaw = particle.pose()(2);

    // Detect code unwrapping and add offset
    if (particle.prevyaw() - curYaw > yaw_unwrap_threshold) {
      // Increment revolutions of cone
      particle.incRev();
      // Print debug info
      ROS_DEBUG_STREAM("+Previous yaw: "
                       << particle.prevyaw() << " Current yaw: " << curYaw
                       << " Diff: " << abs(particle.prevyaw() - curYaw)
                       << " Rev:" << particle.rev());
    } else if (curYaw - particle.prevyaw() > yaw_unwrap_threshold) {
      // Increment revolutions of cone
      particle.decRev();
      // Print debug info
      ROS_DEBUG_STREAM("-Previous yaw: "
                       << particle.prevyaw() << " Current yaw: " << curYaw
                       << " Diff: " << abs(particle.prevyaw() - curYaw)
                       << " Rev:" << particle.rev());
    }

    // Correct yaw by offsetting
    curYaw += particle.rev() * 2 * M_PI;

    if (abs(particle.pose()(2) - particle.prevyaw()) > yaw_unwrap_threshold)
      // Print corrected yaw
      ROS_DEBUG_STREAM("Corrected yaw: " << curYaw);
    this->diagPublisher->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::OK, "Yaw correction",
        "Corrected yaw: " + std::to_string(curYaw));

    x += particle.pose()(0) * w;
    y += particle.pose()(1) * w;
    yaw += curYaw * w;
    // totalW += w; linting

    particle.setPrevyaw(particle.pose()(2));

    if (w > maxW) {
      maxW = w;
    }
  }

  Vector3f pose(x, y, yaw);

  boost::array<double, 36> poseCovariance;

  poseCovariance[0] = calculate_covariance(poseX, poseX);
  poseCovariance[1] = calculate_covariance(poseX, poseY);
  poseCovariance[5] = calculate_covariance(poseYaw, poseX);
  poseCovariance[7] = calculate_covariance(poseY, poseY);
  poseCovariance[11] = calculate_covariance(poseYaw, poseY);
  poseCovariance[35] = calculate_covariance(poseYaw, poseYaw);

  poseCovariance[6] = poseCovariance[1];
  poseCovariance[30] = poseCovariance[5];
  poseCovariance[31] = poseCovariance[11];

  // Odometry message
  nav_msgs::Odometry odom;

  odom.header.stamp = lookupTime;
  odom.header.frame_id = this->map_frame;
  odom.child_frame_id = this->slam_base_link_frame;

  odom.pose.pose.position.x = pose(0);
  odom.pose.pose.position.y = pose(1);

  tf2::Quaternion quat;
  quat.setRPY(0, 0, pose(2));

  odom.pose.pose.orientation.x = quat.getX();
  odom.pose.pose.orientation.y = quat.getY();
  odom.pose.pose.orientation.z = quat.getZ();
  odom.pose.pose.orientation.w = quat.getW();

  odom.pose.covariance = poseCovariance;

  tf2::Transform transform(quat, tf2::Vector3(pose(0), pose(1), 0));

  geometry_msgs::TransformStamped transformMsg;
  transformMsg.header.stamp = lookupTime;
  transformMsg.header.stamp = this->map_frame;
  transformMsg.header.stamp = ros::Time::now();
  transformMsg.child_frame_id = this->slam_base_link_frame;

  transformMsg.transform.translation.x = pose(0);
  transformMsg.transform.translation.y = pose(1);

  transformMsg.transform.rotation.x = quat.getX();
  transformMsg.transform.rotation.y = quat.getY();
  transformMsg.transform.rotation.z = quat.getZ();
  transformMsg.transform.rotation.w = quat.getW();

  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transformMsg);

  // Publish everything!
  this->odomPublisher.publish(odom);
}
} // namespace slam