// #define _GLIBCXX_USE_CXX11_ABI 0
#include "mcl.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/message_filter.h>

#include "particle.hpp"

#include <Eigen/Dense>

#include "ros/ros.h"
#include "ros/console.h"

#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include "kdtree.h"
#include "kdtreepoint.h"
#include "dbscan.hpp"
#include "helpers.hpp"

#include <nav_msgs/Odometry.h>

#include <string>

#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>
#include <ugr_msgs/ObservationWithCovariance.h>

#include <cmath>

using namespace std;
using namespace Eigen;

namespace slam
{

  MCL::MCL(ros::NodeHandle &n) : tfListener(tfBuffer),
                                 n(n),
                                 base_link_frame(n.param<string>("base_link_frame", "ugr/car_base_link")),
                                 slam_world_frame(n.param<string>("slam_world_frame", "ugr/slam_odom")),
                                 world_frame(n.param<string>("world_frame", "ugr/car_odom")),
                                 particle_count(n.param<int>("particle_count", 100)),
                                 effective_particle_count(n.param<int>("effective_particle_count", 75)),
                                 eps(n.param<double>("eps", 2.0)),
                                 max_range(n.param<double>("max_range", 15)),
                                 max_half_fov(n.param<double>("max_half_angle", 60 * 0.0174533)),
                                 observe_dt(n.param<double>("observe_dt", 0.2)),
                                 prev_state({0, 0, 0}),
                                 Q(2, 2),
                                 R(2, 2),
                                 particles(vector<Particle>(particle_count)),
                                 yaw_unwrap_threshold(n.param<float>("yaw_unwrap_threshold", M_PI * 1.3)),
                                 tf2_filter(obs_sub, tfBuffer, base_link_frame, 1, 0)
  {

    this->odomPublisher = n.advertise<nav_msgs::Odometry>("/output/odom", 5);

    obs_sub.subscribe(n, "/input/observations", 1);
    tf2_filter.registerCallback(boost::bind(&MCL::handleObservations, this, _1));

    vector<double> QAsVector;
    vector<double> RAsVector;

    n.param<vector<double>>("input_noise", QAsVector, {0.1, 0.0, 0.0, 0.02});
    n.param<vector<double>>("measurement_covariance", RAsVector, {0.3, 0.0, 0.0, 0.05});

    if (QAsVector.size() != 4)
      throw invalid_argument("Q (measurement_covariance) Must be a vector of size 4");

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
  }

  /**
   * Applies the motion update to a given particle
   *
   * Args:
   *  Particle: the particle to move
   *  dDist: relative distance moved (euclidian)
   *  dYaw: relative yaw change
   */
  void MCL::motion_update(Particle &particle, double dDist, double dYaw)
  {

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
        xv(1) + dDist * sin(dYaw + xv(2)),
        pi_to_pi2(xv(2) + dYaw);
    particle.setPose(xv_temp);
  }

  /**
   * Converts landmark(s) to observation(s) and back.
   *
   * An observation is relative to the sensor in polar coordinates (range, bearing)
   * A landmark is absolute to the world frame in cartesian coordinates (x, y)
   *
   * Argument signature is always (input, output, robot pose)
   */
  void MCL::landmarks_to_observations(vector<VectorXf> &lm, vector<VectorXf> &obs, VectorXf &pose)
  {
    for (int i = 0; i < lm.size(); i++)
    {
      VectorXf observation(2);
      this->landmark_to_observation(lm[i], observation, pose);
      obs.push_back(observation);
    }
  }
  void MCL::landmark_to_observation(VectorXf &lm, VectorXf &obs, VectorXf &pose)
  {
    float dx = lm(0) - pose(0);
    float dy = lm(1) - pose(1);

    obs(0) = pow(pow(dx, 2) + pow(dy, 2), 0.5);
    obs(1) = atan2(dy, dx) - pose(2);
  }
  void MCL::observations_to_landmarks(vector<VectorXf> &obs, vector<VectorXf> &lm, VectorXf &pose)
  {
    for (int i = 0; i < lm.size(); i++)
    {
      VectorXf landmark(2);
      this->observation_to_landmark(obs[i], landmark, pose);
      lm.push_back(landmark);
    }
  }
  void MCL::observation_to_landmark(VectorXf &obs, VectorXf &lm, VectorXf &pose)
  {
    lm(0) = pose(0) + obs(0) * cos(pose(2) + obs(1));
    lm(1) = pose(1) + obs(0) * sin(pose(2) + obs(1));
  }

  /**
   * Build associations between observation messages and the currently set map.
   *
   * Args:
   *  particle[in]: the particle to build associations for
   *  observations[in]: the raw messages (so relative to base link) that need to be associated
   *  landmarks[out]: the landmark means (coming from observations) that were actually 'associated'
   *  indices[out]: the indices of landmarks on the current map that were 'associated'
   */
  void MCL::build_associations(Particle &particle, ugr_msgs::ObservationWithCovarianceArrayStamped &observations, vector<VectorXf> &landmarks, vector<int> &indices)
  {
    landmarks.clear();

    if (_map.size() == 0)
    {
      return;
    }

    for (auto observation : observations.observations)
    {

      if (observation.observation.observation_class < 0 || observation.observation.observation_class > LANDMARK_CLASS_COUNT)
        continue;

      // Prepare observation
      VectorXf obsAsVector(2);
      obsAsVector << pow(pow(observation.observation.location.x, 2) + pow(observation.observation.location.y, 2), 0.5), atan2(observation.observation.location.y, observation.observation.location.x);

      VectorXf landmark(2);
      this->observation_to_landmark(obsAsVector, landmark, particle.pose());

      const KDTreePoint query(landmark, -1);

      double distance;
      int kdpointIndex = _trees[observation.observation.observation_class].nnSearch(query, &distance);

      if (distance < this->eps)
      {
        KDTreePoint point = (_trees[observation.observation.observation_class].getPoints())[kdpointIndex];
        indices.push_back(point.getId());
        landmarks.push_back(obsAsVector);
      }
    }
  }

  void MCL::resample_particles()
  {
    unsigned long i;
    unsigned long N = particles.size();
    VectorXf w(N);

    for (i = 0; i < N; i++)
    {
      w(i) = particles[i].w();
    }

    float ws = w.sum();
    for (i = 0; i < N; i++)
    {
      particles[i].setW(w(i) / ws);
    }

    float Neff = 0.0;
    vector<int> keep;

    stratified_resample(w, keep, Neff);

    vector<Particle> old_particles = vector<Particle>(particles);
    particles.resize(keep.size());

    if ((Neff < effective_particle_count))
    {
      for (i = 0; i < keep.size(); i++)
      {
        particles[i] = old_particles[keep[i]];
      }

      for (i = 0; i < N; i++)
      {
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
   *  associations: the association data (with which map landmarks does z correspond?)
   *
   * Returns:
   *  the likelihood of these measurements, given the current state of the particle
   */
  double MCL::sensor_update(Particle &particle, vector<VectorXf> &z, vector<int> &associations)
  {

    // First convert map to observations, based on particle
    vector<VectorXf> mapLandmarks;
    for (int association : associations)
    {
      mapLandmarks.push_back(_map[association].pose);
    }

    vector<VectorXf> expectedObservations;
    this->landmarks_to_observations(mapLandmarks, expectedObservations, particle.pose());

    // Calculate weight
    double weight = 1.0;

    for (unsigned int i = 0; i < associations.size(); i++)
    {
      MatrixXf sigma = _map[associations[i]].variance;
      weight *= 1 / sqrt(pow(2 * M_PI, 2) * sigma.determinant()) * exp(-0.5 * (mapLandmarks[i] - expectedObservations[i]).transpose() * sigma.inverse() * (mapLandmarks[i] - expectedObservations[i]));
    }

    return weight;
  }

  /**
   * Handles incoming observations from perception
   *
   * Args:
   *  obs: the observations themselves
   */
  void MCL::handleObservations(const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs)
  {

    // Backwards time detection
    if (this->latestTime - obs->header.stamp.toSec() > 0.5 && this->latestTime > 0.0)
    {
      // Reset
      ROS_WARN("Time went backwards! Resetting fastslam...");

      this->particles.clear();
      for (int i = 0; i < this->particle_count; i++)
      {
        this->particles.push_back(Particle());
      }

      this->prev_state = {0.0, 0.0, 0.0};
      this->latestTime = 0.0;

      return;
    }
    else
    {
      this->latestTime = obs->header.stamp.toSec();
    }

    // Should the particle filter ignore the observations or not?
    chrono::steady_clock::time_point time = chrono::steady_clock::now();
    bool doSensorUpdate = abs(std::chrono::duration_cast<std::chrono::duration<double>>(time - this->prev_time).count()) > this->observe_dt;

    if (doSensorUpdate)
      this->prev_time = time;

    // Step 1: motion update
    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    double time_round;
    t1 = std::chrono::steady_clock::now();

    // Transform observations to base_link_frame at current time
    ugr_msgs::ObservationWithCovarianceArrayStamped transformed_obs;
    transformed_obs.header = obs->header;
    for (auto observation : obs->observations)
    {

      ugr_msgs::ObservationWithCovariance transformed_ob;

      geometry_msgs::PointStamped locStamped;
      locStamped.point = observation.observation.location;
      locStamped.header = obs->header;

      try
      {
        transformed_ob.observation.location = this->tfBuffer.transform<geometry_msgs::PointStamped>(locStamped, this->base_link_frame, ros::Duration(0)).point;
      }
      catch (const exception e)
      {
        ROS_ERROR("observation static transform failed: %s", e.what());
        return;
      }

      // Filter out the observation if it is not "in range". Increases performance
      VectorXf z(2);
      z(0) = pow(pow(transformed_ob.observation.location.x, 2) + pow(transformed_ob.observation.location.y, 2), 0.5);
      z(1) = atan2(transformed_ob.observation.location.y, transformed_ob.observation.location.x);

      if (z(0) > this->max_range || abs(z(1)) > this->max_half_fov)
      {
        continue;
      }

      if (observation.observation.observation_class < 0 || observation.observation.observation_class > LANDMARK_CLASS_COUNT)
        continue;

      transformed_ob.covariance = observation.covariance;
      transformed_ob.observation.observation_class = observation.observation.observation_class;
      transformed_obs.observations.push_back(transformed_ob);
    }

    // Fetch the current pose estimate so that we can estimate dDist and dYaw
    double dDist, dYaw = 0;

    geometry_msgs::TransformStamped car_pose;
    try
    {
      car_pose = this->tfBuffer.lookupTransform(this->world_frame, this->base_link_frame, transformed_obs.header.stamp, ros::Duration(0.1));
    }
    catch (const exception e)
    {
      ROS_ERROR("car_pose transform failed: %s", e.what());
      return;
    }
    const tf2::Quaternion quat(
        car_pose.transform.rotation.x,
        car_pose.transform.rotation.y,
        car_pose.transform.rotation.z,
        car_pose.transform.rotation.w);

    double roll, pitch, yaw, x, y;
    x = car_pose.transform.translation.x;
    y = car_pose.transform.translation.y;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    dYaw = yaw - this->prev_state[2];
    dDist = pow(pow(x - this->prev_state[0], 2) + pow(y - this->prev_state[1], 2), 0.5);

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("Observations preparation took: %f s", time_round);

    for (int j = 0; j < particles.size(); j++)
    {
      Particle &particle = particles[j];
      this->motion_update(particle, dDist, dYaw);
    }

    // Step 2: sensor update (if allowed)
    if (doSensorUpdate)
    {
      t1 = std::chrono::steady_clock::now();

      for (int j = 0; j < particles.size(); j++)
      {

        Particle &particle = particles[j];

        // Get landmark associations (per particle!)
        // 'observations' are the observations that were actually matched
        vector<int> indices;
        vector<VectorXf> observations;
        this->build_associations(particle, transformed_obs, observations, indices);

        // Sensor update
        particle.setW(this->sensor_update(particle, observations, indices));
      }

      // Resampling step
      resample_particles();

      t2 = std::chrono::steady_clock::now();

      time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
      ROS_INFO("FastSLAM1.0 took: %f s. That is %f s per particle", time_round, time_round / this->particles.size());
    }

    // Finalizing
    this->prev_state = {x, y, yaw};

    // Done ! Now produce the output
    this->publishOutput();
  }

  void MCL::publishOutput()
  {

    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    double time_round;

    t1 = std::chrono::steady_clock::now();

    // Calculate statistical mean and covariance of the particle filter (pose)
    // Also gather some other information while we are looping

    vector<float> poseX;
    vector<float> poseY;
    vector<float> poseYaw;

    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;
    float maxW = -10000.0;
    float totalW = 0.0;
    for (int i = 0; i < this->particles.size(); i++)
    {

      Particle &particle = this->particles[i];

      float w = particle.w();

      if (isnan(w) || w < 0.001)
      {
        w = 0.001;
      }

      poseX.push_back(particle.pose()(0));
      poseY.push_back(particle.pose()(1));
      poseYaw.push_back(particle.pose()(2));

      float curYaw = particle.pose()(2);

      // Detect code unwrapping and add offset
      if (particle.prevyaw() - curYaw > yaw_unwrap_threshold)
      {
        // Increment revolutions of cone
        particle.incRev();
        // Print debug info
        ROS_DEBUG_STREAM("+Previous yaw: " << particle.prevyaw() << " Current yaw: " << curYaw << " Diff: " << abs(particle.prevyaw() - curYaw) << " Rev:" << particle.rev());
      }
      else if (curYaw - particle.prevyaw() > yaw_unwrap_threshold)
      {
        // Increment revolutions of cone
        particle.decRev();
        // Print debug info
        ROS_DEBUG_STREAM("-Previous yaw: " << particle.prevyaw() << " Current yaw: " << curYaw << " Diff: " << abs(particle.prevyaw() - curYaw) << " Rev:" << particle.rev());
      }

      // Correct yaw by offsetting
      curYaw += particle.rev() * 2 * M_PI;

      if (abs(particle.pose()(2) - particle.prevyaw()) > yaw_unwrap_threshold)
        // Print corrected yaw
        ROS_DEBUG_STREAM("Corrected yaw: " << curYaw);

      x += particle.pose()(0) * w;
      y += particle.pose()(1) * w;
      yaw += curYaw * w;
      totalW += w;

      particle.setPrevyaw(particle.pose()(2));

      if (w > maxW)
      {
        maxW = w;
      }
    }

    VectorXf pose(3);
    pose << x, y, yaw;

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

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("Output averaging took %f s. That is %f s per particle", time_round, time_round / this->particles.size());

    t1 = std::chrono::steady_clock::now();

    // Odometry message
    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = this->slam_world_frame;
    odom.child_frame_id = this->base_link_frame;

    odom.pose.pose.position.x = pose(0);
    odom.pose.pose.position.y = pose(1);

    tf2::Quaternion quat;
    quat.setRPY(0, 0, pose(2));

    odom.pose.pose.orientation.x = quat.getX();
    odom.pose.pose.orientation.y = quat.getY();
    odom.pose.pose.orientation.z = quat.getZ();
    odom.pose.pose.orientation.w = quat.getW();

    odom.pose.covariance = poseCovariance;

    // TF Transformation
    // This uses the 'inversed frame' principle
    // So the new slam_world_frame is a child of base_link_frame and then the inverse transform is published
    tf2::Transform transform(quat, tf2::Vector3(pose(0), pose(1), 0));
    tf2::Transform invTransform = transform.inverse();

    tf2::Quaternion invQuat = invTransform.getRotation();
    tf2::Vector3 invTranslation = invTransform.getOrigin();

    geometry_msgs::TransformStamped transformMsg;
    transformMsg.header.frame_id = this->base_link_frame;
    transformMsg.header.stamp = ros::Time::now();
    transformMsg.child_frame_id = this->slam_world_frame;

    transformMsg.transform.translation.x = invTranslation.x();
    transformMsg.transform.translation.y = invTranslation.y();

    transformMsg.transform.rotation.x = invQuat.getX();
    transformMsg.transform.rotation.y = invQuat.getY();
    transformMsg.transform.rotation.z = invQuat.getZ();
    transformMsg.transform.rotation.w = invQuat.getW();

    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transformMsg);

    // Publish everything!
    this->odomPublisher.publish(odom);

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("Output publishing took %f s.", time_round);
  }
}