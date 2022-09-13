// #define _GLIBCXX_USE_CXX11_ABI 0
#include "fastslam1.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/message_filter.h>

#include "fastslam_core.hpp"

#include <Eigen/Dense>

#include "ros/ros.h"

#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include "kdtree.h"

#include <nav_msgs/Odometry.h>

#include <string>

#include <ugr_msgs/Observations.h>
#include <ugr_msgs/Observation.h>

#include <cmath>

using namespace std;
using namespace Eigen;

namespace slam
{
  FastSLAM1::FastSLAM1(ros::NodeHandle &n) : tfListener(tfBuffer),
                                             n(n),
                                             base_link_frame(n.param<string>("base_link_frame", "ugr/car_base_link")),
                                             slam_world_frame(n.param<string>("slam_world_frame", "ugr/slam_odom")),
                                             world_frame(n.param<string>("world_frame", "ugr/car_odom")),
                                             particle_count(n.param<int>("particle_count", 100)),
                                             effective_particle_count(n.param<int>("effective_particle_count", 75)),
                                             eps(n.param<double>("eps", 2.0)),
                                             expected_range(n.param<double>("expected_range", 15)),
                                             expected_half_fov(n.param<double>("expected_half_angle", 60 * 0.0174533)),
                                             max_range(n.param<double>("max_range", 15)),
                                             max_half_fov(n.param<double>("max_half_angle", 60 * 0.0174533)),
                                             acceptance_score(n.param<double>("acceptance_score", 3.0)),
                                             penalty_score(n.param<double>("penalty_score", -1)),
                                             minThreshold(n.param<double>("discard_score", -2.0)),
                                             prev_state({0, 0, 0}),
                                             Q(2, 2),
                                             R(2, 2),
                                             particles(vector<Particle>(particle_count)),
                                             tf2_filter(obs_sub, tfBuffer, base_link_frame, 1, 0)
  {

    this->globalPublisher = n.advertise<ugr_msgs::Observations>("/output/map", 5);
    this->localPublisher = n.advertise<ugr_msgs::Observations>("/output/observations", 5);
    this->odomPublisher = n.advertise<nav_msgs::Odometry>("/output/odom", 5);

    obs_sub.subscribe(n, "/input/observations", 1);
    tf2_filter.registerCallback(boost::bind(&FastSLAM1::handleObservations, this, _1));

    vector<double> QAsVector;
    vector<double> RAsVector;

    n.param<vector<double>>("input_noise", QAsVector, {0.1, 0.0, 0.0, 0.02});
    n.param<vector<double>>("measurement_covariance", RAsVector, {0.3, 0.0, 0.0, 0.05});

    if (QAsVector.size() != 4)
      throw invalid_argument("Q (measurement_covariance) Must be a vector of size 4");

    if (RAsVector.size() != 4)
      throw invalid_argument("R (input_noise) Must be a vector of size 4");

    if (penalty_score > 0.0)
      throw invalid_argument("penalty_score should be less than zero");

    this->Q(0, 0) = pow(QAsVector[0], 2);
    this->Q(0, 1) = pow(QAsVector[1], 2);
    this->Q(1, 0) = pow(QAsVector[2], 2);
    this->Q(1, 1) = pow(QAsVector[3], 2);

    this->R(0, 0) = pow(RAsVector[0], 2);
    this->R(0, 1) = pow(RAsVector[1], 2);
    this->R(1, 0) = pow(RAsVector[2], 2);
    this->R(1, 1) = pow(RAsVector[3], 2);
  }

  void FastSLAM1::predict(Particle &particle, double dDist, double dYaw)
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
    VectorXf xv = particle.xv();
    VectorXf xv_temp(3);
    xv_temp << xv(0) + dDist * cos(dYaw + xv(2)),
        xv(1) + dDist * sin(dYaw + xv(2)),
        pi_to_pi2(xv(2) + dYaw);
    particle.setXv(xv_temp);
  }

  void FastSLAM1::landmarks_to_observations(vector<VectorXf> &lm, vector<VectorXf> &obs, VectorXf &pose)
  {
    for (int i = 0; i < lm.size(); i++)
    {
      VectorXf observation(2);
      this->landmark_to_observation(lm[i], observation, pose);
      obs.push_back(observation);
    }
  }

  void FastSLAM1::landmark_to_observation(VectorXf &lm, VectorXf &obs, VectorXf &pose)
  {
    float dx = lm(0) - pose(0);
    float dy = lm(1) - pose(1);

    obs(0) = pow(pow(dx, 2) + pow(dy, 2), 0.5);
    obs(1) = atan2(dy, dx) - pose(2);
  }

  void FastSLAM1::observations_to_landmarks(vector<VectorXf> &obs, vector<VectorXf> &lm, VectorXf &pose)
  {
    for (int i = 0; i < lm.size(); i++)
    {
      VectorXf landmark(2);
      this->observation_to_landmark(obs[i], landmark, pose);
      lm.push_back(landmark);
    }
  }

  void FastSLAM1::observation_to_landmark(VectorXf &obs, VectorXf &lm, VectorXf &pose)
  {
    lm(0) = pose(0) + obs(0) * cos(pose(2) + obs(1));
    lm(1) = pose(1) + obs(0) * sin(pose(2) + obs(1));
  }

  void FastSLAM1::build_associations(Particle &particle, ugr_msgs::Observations &observations, vector<VectorXf> &knownLandmarks, vector<VectorXf> &newLandmarks, vector<int> &knownIndices, vector<int> &knownClasses, vector<int> &newClasses)
  {

    // Make a kdtree of the current particle
    LandmarkSearchResult result;
    vector<KDTreePoint> kdtreePoints;

    if (particle.xf().size() == 0)
    {
      result.index = -1;
    }
    else
    {
      for (int i = 0; i < particle.xf().size(); i++)
      {
        if (particle.metadata()[i].score > this->minThreshold)
          kdtreePoints.push_back(KDTreePoint(particle.xf()[i], i));
      }
    }

    kdt::KDTree<KDTreePoint> tree(kdtreePoints);


    for (auto observation : observations.observations)
    {

      VectorXf obsAsVector(2);
      obsAsVector << pow(pow(observation.location.x, 2) + pow(observation.location.y, 2), 0.5), atan2(observation.location.y, observation.location.x);

      VectorXf landmark(2);
      this->observation_to_landmark(obsAsVector, landmark, particle.xv());

      const KDTreePoint query(landmark, -1);

      if (result.index != -1)
      {
        int kdpointIndex = tree.nnSearch(query, &(result.distance));
        result.index = kdtreePoints[kdpointIndex].getId();
      }

      if (result.index == -1 || result.distance > this->eps)
      {
        newLandmarks.push_back(landmark);
        newClasses.push_back(observation.observation_class);
      }
      else
      {
        knownLandmarks.push_back(particle.xf()[result.index]);
        knownIndices.push_back(result.index);
        knownClasses.push_back(observation.observation_class);
      }
    }
  }

  double FastSLAM1::compute_particle_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R)
  {
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;
    vector<VectorXf> zp;

    // process each feature, incrementally refine proposal distribution
    compute_jacobians(particle, idf, R, zp, &Hv, &Hf, &Sf);

    vector<VectorXf> v;

    for (unsigned long j = 0; j < z.size(); j++)
    {
      VectorXf v_j = z[j] - zp[j];
      v_j[1] = pi_to_pi(v_j[1]);
      v.push_back(v_j);
    }

    double w = 1.0;

    MatrixXf S;
    float den, num;
    for (unsigned long i = 0; i < z.size(); i++)
    {
      S = Sf[i];
      den = 2 * M_PI * sqrt(S.determinant());
      num = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
      w = w * num / den;
    }
    return w;
  }

  void FastSLAM1::handleObservations(const ugr_msgs::ObservationsConstPtr &obs)
  {

    // Transform the observations to the base_link frame

    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    double time_round;

    t1 = std::chrono::steady_clock::now();
    ugr_msgs::Observations transformed_obs;
    transformed_obs.header = obs->header;

    for (auto observation : obs->observations)
    {

      ugr_msgs::Observation transformed_ob;

      geometry_msgs::PointStamped locStamped;
      locStamped.point = observation.location;
      locStamped.header = obs->header;

      try
      {
        transformed_ob.location = this->tfBuffer.transform<geometry_msgs::PointStamped>(locStamped, this->base_link_frame, ros::Duration(0)).point;
      }
      catch (const exception e)

      {
        ROS_ERROR("observation static transform failed: %s", e.what());
        return;
      }

      // Filter out the observation if it is not "in range"
      VectorXf z(2);
      z(0) = pow(pow(transformed_ob.location.x, 2) + pow(transformed_ob.location.y, 2), 0.5);
      z(1) = atan2(transformed_ob.location.y, transformed_ob.location.x);

      if (z(0) > this->max_range || abs(z(1)) > this->max_half_fov)
      {
        continue;
      }

      transformed_ob.observation_class = observation.observation_class;
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

    vector<vector<int>> knownObsIndicesVector;
    vector<int> newLmsCounts;

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("Observations preparation took: %f s", time_round);

    t1 = std::chrono::steady_clock::now();

    for (int j = 0; j < particles.size(); j++)
    {

      Particle &particle = particles[j];

      //---- Predict step -----//
      this->predict(particle, dDist, dYaw);

      //---- Observe step ----//

      // Get landmark associations (per particle!)
      vector<VectorXf> knownLms, newLms;
      vector<int> knownClasses;
      vector<int> newClasses;

      vector<int> knownObsIndices;

      this->build_associations(particle, transformed_obs, knownLms, newLms, knownObsIndices, knownClasses, newClasses);

      //----- Update step -----//

      newLmsCounts.push_back(newLms.size());
      if (!newLms.empty())
      {

        vector<VectorXf> z;
        this->landmarks_to_observations(newLms, z, particle.xv());

        add_feature(particle, z, this->R, newClasses);
      }

      if (!knownLms.empty())
      {

        vector<VectorXf> z;
        this->landmarks_to_observations(knownLms, z, particle.xv());

        double w = this->compute_particle_weight(particle, z, knownObsIndices, this->R);
        particle.setW(particle.w() * w);

        feature_update(particle, z, knownObsIndices, this->R, knownClasses);
      }

      knownObsIndicesVector.push_back(knownObsIndices);
    }

    //---- Resample step ----//
    resample_particles(this->particles, this->effective_particle_count, 1);

    // Finalizing
    this->prev_state = {x, y, yaw};

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("FastSLAM1.0 took: %f s. That is %f s per particle", time_round, time_round / this->particles.size());

    t1 = std::chrono::steady_clock::now();
    // Check which cones should have been seen but were not and lower their score
    for (int k = 0; k < particles.size(); k++)
    {

      Particle &particle = particles[k];
      vector<int> &indices = knownObsIndicesVector[k];

      vector<VectorXf> zs;
      this->landmarks_to_observations(particle.xf(), zs, particle.xv());

      for (int i = 0; i < zs.size() - newLmsCounts[k]; i++)
      {
        if (zs[i](0) < this->expected_range && abs(zs[i](1)) < this->expected_half_fov && count(indices.begin(), indices.end(), i) == 0)
        {
          LandmarkMetadata meta = particle.metadata()[i];
          meta.score += penalty_score;
          particle.setMetadatai(i, meta);
        }
      }
    }
    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("FP Filter took %f s. That is %f s per particle", time_round, time_round / this->particles.size());

    // Done ! Now produce the output
    this->publishOutput();
  }

  void FastSLAM1::publishOutput()
  {

    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    double time_round;

    t1 = std::chrono::steady_clock::now();

    // Average (weighted) all the poses and cone positions to get the final estimate

    vector<VectorXf> lmMeans;
    vector<LandmarkMetadata> lmMetadatas;
    vector<float> lmTotalWeight;
    vector<int> contributions;

    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;
    float totalW = 0.0;
    float maxW = -10000.0;
    Particle& bestParticle = this->particles[0];
    for (int i = 0; i < this->particles.size(); i++)
    {

      Particle& particle = this->particles[i];

      float w = particle.w();

      if (isnan(w) || w < 0.001)
      {
        w = 0.001;
      }

      totalW += w;

      x += particle.xv()(0) * w;
      y += particle.xv()(1) * w;
      yaw += particle.xv()(2) * w;

      if (w > maxW) {
        maxW = w;
        bestParticle = particle;
      }
    }

    lmMeans = bestParticle.xf();
    lmMetadatas = bestParticle.metadata();

    VectorXf pose(3);
    pose << x / totalW, y / totalW, yaw / totalW;

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("Output averaging took %f s. That is %f s per particle", time_round, time_round / this->particles.size());

    t1 = std::chrono::steady_clock::now();

    ROS_INFO("Number of landmarks (total): %d", lmMeans.size());

    // Get the landmarks that have a high enough score
    vector<VectorXf> filteredLandmarks;
    vector<LandmarkMetadata> filteredMeta;
    vector<int> filteredLandmarkIndices;

    for (int i = 0; i < lmMeans.size(); i++)
    {
      if (lmMetadatas[i].score >= this->acceptance_score)
      {
        filteredMeta.push_back(lmMetadatas[i]);
        filteredLandmarks.push_back(lmMeans[i]);
        filteredLandmarkIndices.push_back(i);
      }
    }

    ROS_INFO("Number of actual landmarks: %d", filteredLandmarks.size());

    // Create the observation_msgs things
    ugr_msgs::Observations global;
    ugr_msgs::Observations local;
    global.header.frame_id = this->slam_world_frame;
    global.header.stamp = ros::Time::now();
    local.header.frame_id = this->base_link_frame;
    local.header.stamp = ros::Time::now();

    for (int i = 0; i < filteredLandmarks.size(); i++)
    {

      ugr_msgs::Observation global_ob;
      ugr_msgs::Observation local_ob;

      float rounded_float = round((float)filteredMeta[i].classSummation / (float)filteredMeta[i].classSummationCount);
      global_ob.observation_class = uint8_t(rounded_float);
      local_ob.observation_class = int8_t(rounded_float);

      global_ob.location.x = filteredLandmarks[i](0);
      global_ob.location.y = filteredLandmarks[i](1);
      global.observations.push_back(global_ob);
      VectorXf z(2);
      this->landmark_to_observation(filteredLandmarks[i], z, pose);

      local_ob.location.x = z(0) * cos(z(1));
      local_ob.location.y = z(0) * sin(z(1));
      local.observations.push_back(local_ob);
    }

    // Odometry message (for correction of state estimation)
    // So from world_frame to base_link_frame
    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = this->world_frame;
    odom.child_frame_id = this->base_link_frame;

    odom.pose.pose.position.x = pose(0);
    odom.pose.pose.position.y = pose(1);

    tf2::Quaternion quat;
    quat.setRPY(0, 0, pose(2));

    odom.pose.pose.orientation.x = quat.getX();
    odom.pose.pose.orientation.y = quat.getY();
    odom.pose.pose.orientation.z = quat.getZ();
    odom.pose.pose.orientation.w = quat.getW();

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
    this->globalPublisher.publish(global);
    this->localPublisher.publish(local);
    this->odomPublisher.publish(odom);

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("Output publishing took %f s.", time_round);
  }
}