// #define _GLIBCXX_USE_CXX11_ABI 0
#include "fastslam1.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "fastslam_core.hpp"

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

#include "dbscan.hpp"
#include "kdtree.h"

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
FastSLAM1::FastSLAM1(ros::NodeHandle &n)
    : tfListener(tfBuffer), n(n),
      base_link_frame(n.param<string>("base_link_frame", "ugr/car_base_link")),
      slam_base_link_frame(
          n.param<string>("slam_base_link_frame", "ugr/slam_base_link")),
      world_frame(n.param<string>("world_frame", "ugr/car_odom")),
      map_frame(n.param<string>("map_frame", "ugr/map")),
      lidar_frame(n.param<string>("lidar_frame", "os_sensor")),
      particle_count(n.param<int>("particle_count", 100)),
      post_clustering(n.param<bool>("post_clustering", false)),
      doSynchronous(n.param<bool>("synchronous", true)),
      effective_particle_count(n.param<int>("effective_particle_count", 75)),
      min_clustering_point_count(
          n.param<int>("min_clustering_point_count", 30)),
      clustering_eps(n.param<double>("clustering_eps", 0.5)),
      belief_factor(n.param<double>("belief_factor", 2.0)),
      observe_prob(n.param<double>("observe_prob", 0.9)), prev_state({0, 0, 0}),
      publish_rate(n.param<double>("publish_rate", 3.0)),
      average_output_pose(n.param<bool>("average_output_pose", true)), Q(3, 3),
      R(2, 2),
      yaw_unwrap_threshold(n.param<float>("yaw_unwrap_threshold", M_PI * 1.3)),
      tf2_filter(obs_sub, tfBuffer, base_link_frame, 1, 0) {
  lidarOptions = {};
  lidarOptions.eps = n.param<double>("lidar_eps", 0.5);
  lidarOptions.max_range = n.param<double>("lidar_max_range", 15);
  lidarOptions.max_half_fov =
      n.param<double>("lidar_max_half_angle", 60 * 0.0174533);
  lidarOptions.expected_range = n.param<double>("lidar_expected_range", 15);
  lidarOptions.expected_half_fov =
      n.param<double>("lidar_expected_half_angle", 60 * 0.0174533);
  lidarOptions.acceptance_score =
      n.param<double>("lidar_acceptance_score", 5.0);
  lidarOptions.penalty_score = n.param<double>("lidar_penalty_score", -0.5);
  lidarOptions.minThreshold = n.param<double>("lidar_discard_score", -2.0);

  cameraOptions = {};
  cameraOptions.eps = n.param<double>("camera_eps", 0.5);
  cameraOptions.max_range = n.param<double>("camera_max_range", 15);
  cameraOptions.max_half_fov =
      n.param<double>("camera_max_half_andle", 60 * 0.0174533);
  cameraOptions.expected_range = n.param<double>("camera_expected_range", 15);
  cameraOptions.expected_half_fov =
      n.param<double>("camera_expected_half_angle", 60 * 0.0174533);
  cameraOptions.acceptance_score =
      n.param<double>("camera_acceptance_score", 5.0);
  cameraOptions.penalty_score = n.param<double>("camera_penalty_score", -0.5);
  cameraOptions.minThreshold = n.param<double>("camera_discard_score", -2.0);

  // Initialize map Service Client
  string SetMap_service =
      n.param<string>("SetMap_service", "/ugr/srv/slam_map_server/set");
  this->setmap_srv_client =
      n.serviceClient<slam_controller::SetMap::Request>(SetMap_service, true);

  this->globalmap_namespace = n.param<string>("globalmap_namespace", "global");
  this->localmap_namespace = n.param<string>("localmap_namespace", "local");

  this->odomPublisher = n.advertise<nav_msgs::Odometry>("/output/odom", 5);
  this->particlePosePublisher =
      n.advertise<geometry_msgs::PoseArray>("/output/particles", 5);

  this->diagPublisher = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n, "SLAM FastSLAM1.0"));

  obs_sub.subscribe(n, "/input/observations", 1);
  tf2_filter.registerCallback(
      boost::bind(&FastSLAM1::handleObservations, this, _1));

  this->prev_predict_time = chrono::steady_clock::now();

  gotFirstObservations = false;

  vector<double> QAsVector;
  vector<double> RAsVector;

  n.param<vector<double>>("input_noise", QAsVector,
                          {0.1, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.05});
  n.param<vector<double>>("measurement_covariance", RAsVector,
                          {0.3, 0.0, 0.0, 0.05});

  if (QAsVector.size() != 9)
    throw invalid_argument(
        "Q (measurement_covariance) Must be a vector of size 9");

  if (RAsVector.size() != 4)
    throw invalid_argument("R (input_noise) Must be a vector of size 4");

  if (this->options->penalty_score > 0.0)
    throw invalid_argument("penalty_score should be less than zero");

  this->Q(0, 0) = pow(QAsVector[0], 2);
  this->Q(0, 1) = pow(QAsVector[1], 2);
  this->Q(0, 2) = pow(QAsVector[2], 2);
  this->Q(1, 0) = pow(QAsVector[3], 2);
  this->Q(1, 1) = pow(QAsVector[4], 2);
  this->Q(1, 2) = pow(QAsVector[5], 2);
  this->Q(2, 0) = pow(QAsVector[6], 2);
  this->Q(2, 1) = pow(QAsVector[7], 2);
  this->Q(2, 2) = pow(QAsVector[8], 2);

  this->R(0, 0) = pow(RAsVector[0], 2);
  this->R(0, 1) = pow(RAsVector[1], 2);
  this->R(1, 0) = pow(RAsVector[2], 2);
  this->R(1, 1) = pow(RAsVector[3], 2);

  double stand_dev = n.param<double>("setup_stand_dev", 0.1);
  this->particles.clear();
  for (int i = 0; i < this->particle_count; i++) {
    this->particles.push_back(Particle(stand_dev));
  }
  firstRound = true;
}

void FastSLAM1::predict(Particle &particle, double dDist, double dYaw,
                        double dt) {

  // ROS_INFO("dDist: %f", dDist);

  // Add noise
  VectorXf A(3);
  A(0) = dDist;
  A(1) = dYaw;
  A(2) = 0.0;
  VectorXf VG(3);
  MatrixXf Q = this->Q * dt;
  VG = multivariate_gauss(A, Q, 1);
  dDist = VG(0);
  dYaw = VG(1);

  // predict state
  VectorXf xv = particle.xv();
  VectorXf xv_temp(3);
  xv_temp << xv(0) + dDist * cos(dYaw + xv(2)),
      xv(1) + dDist * sin(dYaw + xv(2)), pi_to_pi2(xv(2) + dYaw + VG(2));
  particle.setXv(xv_temp);
}

void FastSLAM1::landmarks_to_observations(vector<VectorXf> &lm,
                                          vector<VectorXf> &obs,
                                          VectorXf &pose) {
  for (int i = 0; i < lm.size(); i++) {
    VectorXf observation(2);
    this->landmark_to_observation(lm[i], observation, pose);
    obs.push_back(observation);
  }
}

void FastSLAM1::landmark_to_observation(VectorXf &lm, VectorXf &obs,
                                        VectorXf &pose) {
  float dx = lm(0) - pose(0);
  float dy = lm(1) - pose(1);

  obs(0) = pow(pow(dx, 2) + pow(dy, 2), 0.5);
  obs(1) = atan2(dy, dx) - pose(2);
}

void FastSLAM1::observations_to_landmarks(vector<VectorXf> &obs,
                                          vector<VectorXf> &lm,
                                          VectorXf &pose) {
  for (int i = 0; i < lm.size(); i++) {
    VectorXf landmark(2);
    this->observation_to_landmark(obs[i], landmark, pose);
    lm.push_back(landmark);
  }
}

void FastSLAM1::observation_to_landmark(VectorXf &obs, VectorXf &lm,
                                        VectorXf &pose) {
  lm(0) = pose(0) + obs(0) * cos(pose(2) + obs(1));
  lm(1) = pose(1) + obs(0) * sin(pose(2) + obs(1));
}

void FastSLAM1::build_associations(
    Particle &particle,
    ugr_msgs::ObservationWithCovarianceArrayStamped &observations,
    vector<VectorXf> &knownLandmarks, vector<VectorXf> &newLandmarks,
    vector<int> &knownIndices, vector<int> &knownClasses,
    vector<int> &newClasses, vector<float> &knownBeliefs,
    vector<float> &newBeliefs) {

  // Make a kdtree of the current particle
  LandmarkSearchResult result;
  std::vector<VectorXf> vectorsToConsider;
  std::vector<int> indices;

  if (particle.xf().size() == 0) {
    result.index = -1;
  } else {
    for (int i = particle.xf().size() - 1; i >= 0; i--) {
      if (particle.metadata()[i].score > this->options->minThreshold) {
        vectorsToConsider.push_back(particle.xf()[i]);
        indices.push_back(i);
      }
    }
  }

  for (auto observation : observations.observations) {

    VectorXf obsAsVector(2);
    obsAsVector << pow(pow(observation.observation.location.x, 2) +
                           pow(observation.observation.location.y, 2),
                       0.5),
        atan2(observation.observation.location.y,
              observation.observation.location.x);

    VectorXf landmark(2);
    this->observation_to_landmark(obsAsVector, landmark, particle.xv());

    // Loop over landmarks until close enough one found (<eps)
    bool found = false;
    int index;
    for (int i = 0; i < vectorsToConsider.size(); i++) {
      float distance = pow(landmark[0] - vectorsToConsider[i](0), 2) +
                       pow(landmark[1] - vectorsToConsider[i](1), 2);

      if (distance < pow(this->options->eps, 2)) {
        index = indices[i];
        found = true;
        break;
      }
    }

    if (!found) {
      newLandmarks.push_back(landmark);
      newClasses.push_back(observation.observation.observation_class);
      newBeliefs.push_back(observation.observation.belief);
    } else {
      knownLandmarks.push_back(particle.xf()[index]);
      knownIndices.push_back(index);
      knownClasses.push_back(observation.observation.observation_class);
      knownBeliefs.push_back(observation.observation.belief);
    }
  }
}

double FastSLAM1::compute_particle_weight(Particle &particle,
                                          vector<VectorXf> &z, vector<int> &idf,
                                          MatrixXf &R, vector<VectorXf> &zp,
                                          vector<MatrixXf> &Hv,
                                          vector<MatrixXf> &Hf,
                                          vector<MatrixXf> &Sf) {

  vector<VectorXf> v;

  for (unsigned long j = 0; j < z.size(); j++) {
    VectorXf v_j = z[j] - zp[j];
    v_j[1] = pi_to_pi(v_j[1]);
    v.push_back(v_j);
  }

  double w = 1.0;

  MatrixXf S;
  for (unsigned long i = 0; i < z.size(); i++) {
    S = Sf[i];
    float den = 2 * M_PI * sqrt(S.determinant());
    float num = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
    w = w * num / den;
  }
  return w;
}

void FastSLAM1::handleObservations(
    const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs) {
  // change options for camera or lidar (simulator geeft base_link_frame mee)
  if (obs->header.frame_id == this->lidar_frame ||
      obs->header.frame_id == this->base_link_frame) {
    options = &lidarOptions;
  } else {
    options = &cameraOptions;
  }

  if (this->latestTime - obs->header.stamp.toSec() > 5.0 &&
      this->latestTime > 0.0) {
    // Reset
    ROS_WARN("Time went backwards! Resetting fastslam...");
    this->diagPublisher->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::WARN, "Backwards time",
        "Fastslam reset (time went backwards)!");

    this->particles.clear();
    for (int i = 0; i < this->particle_count; i++) {
      this->particles.push_back(Particle());
    }

    this->prev_state = {0.0, 0.0, 0.0};
    this->latestTime = 0.0;

    firstRound = true;

    return;
  } else {
    this->latestTime = obs->header.stamp.toSec();
  }

  this->observations = *obs;
  this->gotFirstObservations = true;
  this->updateRound = true;

  if (this->doSynchronous) {
    this->step();
  }
}

void FastSLAM1::step() {

  if (!gotFirstObservations)
    return;

  std::vector<double> times;

  std::chrono::steady_clock::time_point t1;
  std::chrono::steady_clock::time_point t2;

  t1 = std::chrono::steady_clock::now();

  // Sometimes let the particle filter spread out
  bool doObserve = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) <
                   this->observe_prob;

  // Transform the observations to the base_link frame (statically)
  // Only if using the 'asynchronous method' also to current time

  ugr_msgs::ObservationWithCovarianceArrayStamped transformed_obs;
  transformed_obs.header.frame_id = this->base_link_frame;

  if (this->doSynchronous) {
    transformed_obs.header.stamp = this->observations.header.stamp;
  } else {
    transformed_obs.header.stamp = ros::Time::now();
  }

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
    } catch (const exception &e)

    {
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

    if (z(0) > pow(this->options->max_range, 2) ||
        abs(z(1)) > this->options->max_half_fov) {
      continue;
    }

    transformed_ob.covariance = observation.covariance;
    transformed_ob.observation.observation_class =
        observation.observation.observation_class;
    transformed_ob.observation.belief = observation.observation.belief;
    transformed_obs.observations.push_back(transformed_ob);
  }

  // Fetch the current pose estimate (current in: equal to the one of the
  // observations) so that we can estimate dDist and dYaw
  double dDist, dYaw = 0;

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

  dYaw = yaw - this->prev_state[2];

  // Check for reverse
  double drivingAngle = atan2(y - this->prev_state[1], x - this->prev_state[0]);
  double angleDifference = abs(drivingAngle - yaw);
  if (angleDifference > M_PI) {
    angleDifference = 2 * M_PI - angleDifference;
  }

  bool forward = angleDifference < M_PI_2;
  dDist = (forward ? 1 : -1) *
          pow(pow(x - this->prev_state[0], 2) + pow(y - this->prev_state[1], 2),
              0.5);

  // Initial pose mechanism
  if (firstRound) {
    this->prev_state[0] = x;
    this->prev_state[1] = y;
    this->prev_state[2] = yaw;

    this->prev_transform_time = ros::Time::now();

    firstRound = false;
    return;
  }

  vector<vector<int>> knownObsIndicesVector;
  vector<int> newLmsCounts;

  t2 = std::chrono::steady_clock::now();

  times.push_back(
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
          .count());

  double dt =
      (ros::Time::now() - prev_transform_time)
          .toSec(); // std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now()
                    // - this->prev_predict_time).count();
  prev_transform_time = car_pose.header.stamp;

  ROS_INFO("dt %f; dDist %f", dt, dDist);

  if (doObserve) {
    double time_predict = 0.0;
    double time_association = 0.0;
    double time_update = 0.0;

    for (int j = 0; j < particles.size(); j++) {

      t1 = std::chrono::steady_clock::now();

      Particle &particle = particles[j];
      //---- Predict step -----//
      this->predict(particle, dDist, dYaw, dt);

      t2 = std::chrono::steady_clock::now();
      time_predict +=
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

      //---- Observe step ----//

      t1 = std::chrono::steady_clock::now();

      // Get landmark associations (per particle!)
      vector<VectorXf> knownLms, newLms;
      vector<int> knownClasses;
      vector<int> newClasses;

      vector<float> knownBeliefs;
      vector<float> newBeliefs;

      vector<int> knownObsIndices;

      this->build_associations(particle, transformed_obs, knownLms, newLms,
                               knownObsIndices, knownClasses, newClasses,
                               knownBeliefs, newBeliefs);
      t2 = std::chrono::steady_clock::now();
      time_association +=
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

      //----- Update step -----//

      t1 = std::chrono::steady_clock::now();

      newLmsCounts.push_back(newLms.size());
      if (!newLms.empty()) {

        vector<VectorXf> z;
        this->landmarks_to_observations(newLms, z, particle.xv());

        add_feature(particle, z, this->R, newClasses, newBeliefs);
      }

      if (!knownLms.empty()) {

        vector<VectorXf> z;
        this->landmarks_to_observations(knownLms, z, particle.xv());

        vector<VectorXf> zp;
        vector<MatrixXf> Hv;
        vector<MatrixXf> Hf;
        vector<MatrixXf> Sf;

        compute_jacobians(particle, knownObsIndices, this->R, zp, &Hv, &Hf,
                          &Sf);

        double w = this->compute_particle_weight(particle, z, knownObsIndices,
                                                 this->R, zp, Hv, Hf, Sf);
        particle.setW(particle.w() * w);

        feature_update(particle, z, knownObsIndices, this->R, knownClasses,
                       knownBeliefs, zp, Hv, Hf, Sf);
      }

      knownObsIndicesVector.push_back(knownObsIndices);

      t2 = std::chrono::steady_clock::now();
      time_update +=
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();
    }

    times.push_back(time_predict);
    times.push_back(time_association);
    times.push_back(time_update);
  } else {

    for (int j = 0; j < particles.size(); j++) {

      Particle &particle = particles[j];

      //---- Predict step -----//
      this->predict(particle, dDist, dYaw, dt);
    }
  }

  t1 = std::chrono::steady_clock::now();
  //---- Resample step ----//
  resample_particles(this->particles, this->effective_particle_count, 1);
  t2 = std::chrono::steady_clock::now();
  times.push_back(
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
          .count());
  t1 = std::chrono::steady_clock::now();
  // Check which cones should have been seen but were not and lower their score

  if (this->updateRound && doObserve) {
    for (int k = 0; k < particles.size(); k++) {

      Particle &particle = particles[k];
      vector<int> &indices = knownObsIndicesVector[k];

      vector<VectorXf> zs;
      this->landmarks_to_observations(particle.xf(), zs, particle.xv());

      for (int i = 0; i < zs.size() - newLmsCounts[k]; i++) {
        if (zs[i](0) < this->options->expected_range &&
            abs(zs[i](1)) < this->options->expected_half_fov &&
            count(indices.begin(), indices.end(), i) == 0) {
          LandmarkMetadata meta = particle.metadata()[i];
          meta.score += this->options->penalty_score;
          particle.setMetadatai(i, meta);
        }
      }
    }
  }
  t2 = std::chrono::steady_clock::now();
  times.push_back(
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
          .count());
  t1 = std::chrono::steady_clock::now();
  // Finalizing
  this->prev_state = {x, y, yaw};
  this->updateRound = false;
  int size = this->observations.observations.size();
  this->observations.observations.clear();
  t2 = std::chrono::steady_clock::now();
  times.push_back(
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
          .count());

  t1 = std::chrono::steady_clock::now();

  // Done ! Now produce the output
  this->publishOutput(transformed_obs.header.stamp);
  t2 = std::chrono::steady_clock::now();
  times.push_back(
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
          .count());
  double totalTime = std::accumulate(times.begin(), times.end(), 0.0);

  ROS_INFO("Timetable (%d observations, %d particles):", size,
           this->particles.size());
  for (auto const &time : times) {
    ROS_INFO("%.3fms (%.3f%)", time * 1000.0, time * 100.0 / totalTime);
  }
}

void FastSLAM1::publishOutput(ros::Time lookupTime) {

  if (std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - this->prev_publish_time)
          .count() < 1.0 / this->publish_rate) {
    return;
  }

  this->prev_publish_time = std::chrono::steady_clock::now();

  // Calculate statistical mean and covariance of the particle filter (pose)
  // Also gather some other information while we are looping

  vector<VectorXf> samples;
  vector<float> poseX;
  vector<float> poseY;
  vector<float> poseYaw;
  vector<LandmarkMetadata> sampleMetadata;

  Particle &bestParticle = this->particles[0];

  float x = 0.0;
  float y = 0.0;
  float totalW = 0.0;
  float yaw = 0.0;
  float maxW = -10000.0;

  // Skip the first particle due to weird bug
  for (int i = 1; i < this->particles.size(); i++) {

    Particle &particle = this->particles[i];

    float w = particle.w();

    if (isnan(w) || w < 0.001) {
      w = 0.001;
    }

    poseX.push_back(particle.xv()(0));
    poseY.push_back(particle.xv()(1));
    poseYaw.push_back(particle.xv()(2));

    float curYaw = particle.xv()(2);

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

    x += particle.xv()(0) * w;
    y += particle.xv()(1) * w;
    yaw += curYaw * w;
    totalW += w;

    particle.setPrevyaw(particle.xv()(2));

    if (w > maxW) {
      maxW = w;
      bestParticle = this->particles[i];
    }

    for (auto xf : particle.xf()) {
      samples.push_back(xf);
    }
    for (auto metadata : particle.metadata()) {
      sampleMetadata.push_back(metadata);
    }
  }

  // Publish particles as PoseArray
  geometry_msgs::PoseArray particlePoses;
  particlePoses.header.frame_id = this->map_frame;
  for (int i = 0; i < this->particles.size(); i++) {
    geometry_msgs::Pose particlePose;
    particlePose.position.x = poseX[i];
    particlePose.position.y = poseY[i];

    tf2::Quaternion quat;
    quat.setRPY(0, 0, poseYaw[i]);

    particlePose.orientation.x = quat.x();
    particlePose.orientation.y = quat.y();
    particlePose.orientation.z = quat.z();
    particlePose.orientation.w = quat.w();

    particlePoses.poses.push_back(particlePose);
  }
  this->particlePosePublisher.publish(particlePoses);

  VectorXf pose(3);
  if (this->average_output_pose) {
    pose[0] = x / totalW;
    pose[1] = y / totalW;
    pose[2] = yaw / totalW;
  } else {
    pose << bestParticle.xv()(0), bestParticle.xv()(1), bestParticle.xv()(2);
  }
  vector<MatrixXf> positionCovariances;
  vector<VectorXf> lmMeans;
  vector<LandmarkMetadata> lmMetadatas;

  // Just take all the particles from the best particle
  lmMeans = bestParticle.xf();
  lmMetadatas = bestParticle.metadata();
  positionCovariances = bestParticle.Pf();

  ROS_INFO("Number of landmarks (total): %d", lmMeans.size());
  this->diagPublisher->publishDiagnostic(
      node_fixture::DiagnosticStatusEnum::OK, "Total landmarks",
      "#landmarks: " + std::to_string(lmMeans.size()));

  // Get the landmarks that have a high enough score
  vector<VectorXf> filteredLandmarks;
  vector<LandmarkMetadata> filteredMeta;
  vector<int> filteredLandmarkIndices;
  vector<Matrix2f> filteredCovariances;

  for (int i = 0; i < lmMeans.size(); i++) {
    if (lmMetadatas[i].score >= this->options->acceptance_score) {
      filteredCovariances.push_back(positionCovariances[i]);
      filteredMeta.push_back(lmMetadatas[i]);
      filteredLandmarks.push_back(lmMeans[i]);
      filteredLandmarkIndices.push_back(i);
    }
  }

  ROS_INFO("Number of actual landmarks: %d", filteredLandmarks.size());
  this->diagPublisher->publishDiagnostic(
      node_fixture::DiagnosticStatusEnum::OK, "Actual landmarks",
      "#landmarks: " + std::to_string(filteredLandmarks.size()));

  // Create the observation_msgs things
  ugr_msgs::ObservationWithCovarianceArrayStamped global;
  ugr_msgs::ObservationWithCovarianceArrayStamped local;
  global.header.frame_id = this->map_frame;
  global.header.stamp = lookupTime;
  local.header.frame_id = this->slam_base_link_frame;
  local.header.stamp = lookupTime;

  for (int i = 0; i < filteredLandmarks.size(); i++) {

    ugr_msgs::ObservationWithCovariance global_ob;
    ugr_msgs::ObservationWithCovariance local_ob;

    int observation_class = 0;
    float max_count = 0.0;
    float total_count = 0.0;
    for (int j = 0; j < LANDMARK_CLASS_COUNT; j++) {
      total_count += filteredMeta[i].classDetectionCount[j];
      if (filteredMeta[i].classDetectionCount[j] > max_count) {
        observation_class = j;
        max_count = filteredMeta[i].classDetectionCount[j];
      }
    }

    // Calculate the observation class (co)variance
    // First get the odds of a specific class p
    float p[LANDMARK_CLASS_COUNT];
    for (unsigned int j = 0; j < LANDMARK_CLASS_COUNT; j++) {
      p[j] = filteredMeta[i].classDetectionCount[j] /
             (total_count == 0.0 ? 1.0 : total_count);
    }

    float obsClassMean = 0.0;
    for (unsigned int j = 0; j < LANDMARK_CLASS_COUNT; j++) {
      obsClassMean += p[j] * j;
    }

    float obsCovariance = 0.0;
    for (unsigned int j = 0; j < LANDMARK_CLASS_COUNT; j++) {
      obsCovariance += pow(j - obsClassMean, 2) * p[j];
    }

    auto covarianceMatrix = boost::array<double, 9>(
        {filteredCovariances[i](0, 0), filteredCovariances[i](0, 1), 0.0,
         filteredCovariances[i](1, 0), filteredCovariances[i](1, 1), 0.0, 0, 0,
         obsCovariance});

    global_ob.covariance = covarianceMatrix;
    local_ob.covariance = covarianceMatrix;

    global_ob.observation.observation_class = observation_class;
    local_ob.observation.observation_class = observation_class;

    float belief = max(
        min((1 - exp(-1 * belief_factor * filteredMeta[i].score)), 1.0), 0.0);
    global_ob.observation.belief = belief;
    local_ob.observation.belief = belief;

    global_ob.observation.location.x = filteredLandmarks[i](0);
    global_ob.observation.location.y = filteredLandmarks[i](1);
    global.observations.push_back(global_ob);
    VectorXf z(2);
    this->landmark_to_observation(filteredLandmarks[i], z, pose);

    local_ob.observation.location.x = z(0) * cos(z(1));
    local_ob.observation.location.y = z(0) * sin(z(1));
    local.observations.push_back(local_ob);
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

  // Odometry message (for correction of state estimation)
  // So from map_frame to base_link_frame
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

  // Publish odometry
  this->odomPublisher.publish(odom);

  // TF Transformation
  tf2::Transform transform(quat, tf2::Vector3(pose(0), pose(1), 0));

  geometry_msgs::TransformStamped transformMsg;
  transformMsg.header.frame_id = this->map_frame;
  transformMsg.header.stamp = lookupTime;
  transformMsg.child_frame_id = this->slam_base_link_frame;

  transformMsg.transform.translation.x = pose(0);
  transformMsg.transform.translation.y = pose(1);

  transformMsg.transform.rotation.x = quat.getX();
  transformMsg.transform.rotation.y = quat.getY();
  transformMsg.transform.rotation.z = quat.getZ();
  transformMsg.transform.rotation.w = quat.getW();

  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transformMsg);
}
} // namespace slam
