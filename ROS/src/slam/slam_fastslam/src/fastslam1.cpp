// #define _GLIBCXX_USE_CXX11_ABI 0
#include "fastslam1.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/message_filter.h>

#include "fastslam_core.hpp"

#include <Eigen/Dense>

#include "ros/ros.h"
#include "ros/console.h"

#include "ros/service_client.h"
#include <slam_controller/SetMap.h>

#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include "kdtree.h"
#include "dbscan.hpp"

#include <nav_msgs/Odometry.h>

#include <string>

#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>
#include <ugr_msgs/ObservationWithCovariance.h>

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
                                             post_clustering(n.param<bool>("post_clustering", false)),
                                             effective_particle_count(n.param<int>("effective_particle_count", 75)),
                                             min_clustering_point_count(n.param<int>("min_clustering_point_count", 30)),
                                             eps(n.param<double>("eps", 2.0)),
                                             clustering_eps(n.param<double>("clustering_eps", 0.5)),
                                             belief_factor(n.param<double>("belief_factor", 2.0)),
                                             expected_range(n.param<double>("expected_range", 15)),
                                             expected_half_fov(n.param<double>("expected_half_angle", 60 * 0.0174533)),
                                             max_range(n.param<double>("max_range", 15)),
                                             max_half_fov(n.param<double>("max_half_angle", 60 * 0.0174533)),
                                             observe_dt(n.param<double>("observe_dt", 0.2)),
                                             acceptance_score(n.param<double>("acceptance_score", 3.0)),
                                             penalty_score(n.param<double>("penalty_score", -1)),
                                             minThreshold(n.param<double>("discard_score", -2.0)),
                                             prev_state({0, 0, 0}),
                                             Q(2, 2),
                                             R(2, 2),
                                             particles(vector<Particle>(particle_count)),
                                             yaw_unwrap_threshold(n.param<float>("yaw_unwrap_threshold", M_PI * 1.3)),
                                             tf2_filter(obs_sub, tfBuffer, base_link_frame, 1, 0)
  {

    // Initialize map Service Client
    string SetMap_service = n.param<string>("SetMap_service", "/ugr/srv/slam_map_server/set");
    this->setmap_srv_client = n.serviceClient<slam_controller::SetMap::Request>(SetMap_service, true);
    
    this->globalmap_namespace = n.param<string>("globalmap_namespace", "global");
    this->localmap_namespace = n.param<string>("localmap_namespace", "local");

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

  void FastSLAM1::build_associations(Particle &particle, ugr_msgs::ObservationWithCovarianceArrayStamped &observations, vector<VectorXf> &knownLandmarks, vector<VectorXf> &newLandmarks, vector<int> &knownIndices, vector<int> &knownClasses, vector<int> &newClasses)
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
      obsAsVector << pow(pow(observation.observation.location.x, 2) + pow(observation.observation.location.y, 2), 0.5), atan2(observation.observation.location.y, observation.observation.location.x);

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
        newClasses.push_back(observation.observation.observation_class);
      }
      else
      {
        knownLandmarks.push_back(particle.xf()[result.index]);
        knownIndices.push_back(result.index);
        knownClasses.push_back(observation.observation.observation_class);
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

  void FastSLAM1::handleObservations(const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs)
  {

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

    // Sometimes let the particle filter spread out
    chrono::steady_clock::time_point time = chrono::steady_clock::now();
    bool doObserve = abs(std::chrono::duration_cast<std::chrono::duration<double>>(time - this->prev_time).count()) > this->observe_dt;

    if (doObserve)
      this->prev_time = time;

    // Transform the observations to the base_link frame

    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    double time_round;

    t1 = std::chrono::steady_clock::now();
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

      // Filter out the observation if it is not "in range"
      VectorXf z(2);
      z(0) = pow(pow(transformed_ob.observation.location.x, 2) + pow(transformed_ob.observation.location.y, 2), 0.5);
      z(1) = atan2(transformed_ob.observation.location.y, transformed_ob.observation.location.x);

      if (z(0) > this->max_range || abs(z(1)) > this->max_half_fov)
      {
        continue;
      }

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

    vector<vector<int>> knownObsIndicesVector;
    vector<int> newLmsCounts;

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("Observations preparation took: %f s", time_round);

    if (doObserve)
    {

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
    }
    else
    {
      for (int j = 0; j < particles.size(); j++)
      {
        Particle &particle = particles[j];

        //---- Predict step -----//
        this->predict(particle, dDist, dYaw);
      }
    }

    // Finalizing
    this->prev_state = {x, y, yaw};

    // Done ! Now produce the output
    this->publishOutput();
  }

  void FastSLAM1::publishOutput()
  {

    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    double time_round;

    t1 = std::chrono::steady_clock::now();

    // Calculate statistical mean and covariance of the particle filter (pose)
    // Also gather some other information while we are looping

    vector<VectorXf> samples;
    vector<float> poseX;
    vector<float> poseY;
    vector<float> poseYaw;
    vector<LandmarkMetadata> sampleMetadata;

    Particle &bestParticle = this->particles[0];

    vector<int> contributions;

    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;
    float maxW = -10000.0;
    for (int i = 0; i < this->particles.size(); i++)
    {

      Particle &particle = this->particles[i];

      float w = particle.w();

      if (isnan(w) || w < 0.001)
      {
        w = 0.001;
      }

      poseX.push_back(particle.xv()(0));
      poseY.push_back(particle.xv()(1));
      poseYaw.push_back(particle.xv()(2));

      float curYaw = particle.xv()(2);

      // Detect code unwrapping and add offset
      if(particle.prevyaw() - curYaw > yaw_unwrap_threshold) {
        // Increment revolutions of cone
        particle.incRev();
        // Print debug info
        ROS_DEBUG_STREAM("+Previous yaw: " << particle.prevyaw() << " Current yaw: " << curYaw << " Diff: " << abs(particle.prevyaw()-curYaw) << " Rev:" << particle.rev());
      } else if (curYaw - particle.prevyaw() > yaw_unwrap_threshold) {
        // Increment revolutions of cone
        particle.decRev();
        // Print debug info
        ROS_DEBUG_STREAM("-Previous yaw: " << particle.prevyaw() << " Current yaw: " << curYaw << " Diff: " << abs(particle.prevyaw()-curYaw) << " Rev:" << particle.rev());
      }

      // Correct yaw by offsetting
      curYaw += particle.rev() * 2 * M_PI;

      if(abs(particle.xv()(2) - particle.prevyaw()) > yaw_unwrap_threshold)
        // Print corrected yaw
        ROS_DEBUG_STREAM("Corrected yaw: " << curYaw);

      x += particle.xv()(0) * w;
      y += particle.xv()(1) * w;
      yaw += curYaw * w;

      particle.setPrevyaw(particle.xv()(2));

      if (w > maxW)
      {
        maxW = w;
        bestParticle = this->particles[i];
      }

      for (auto xf : particle.xf())
      {
        samples.push_back(xf);
      }
      for (auto metadata : particle.metadata())
      {
        sampleMetadata.push_back(metadata);
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

    vector<MatrixXf> positionCovariances;
    vector<VectorXf> lmMeans;
    vector<LandmarkMetadata> lmMetadatas;

    if (this->post_clustering)
    {
      // Now apply DBSCAN to the samples
      // A cluster is a vector of indices
      vector<vector<unsigned int>> clusters = dbscanVectorXf(samples, this->clustering_eps, this->min_clustering_point_count);

      // Convert the clusters back to landmarks
      for (vector<unsigned int> cluster : clusters)
      {

        VectorXf lmMean(2);
        LandmarkMetadata lmMetadata;

        float totalP = 0.0;

        for (unsigned int index : cluster)
        {
          lmMean[0] += samples[index][0];
          lmMean[1] += samples[index][1];

          for (int i = 0; i < LANDMARK_CLASS_COUNT; i++)
          {
            lmMetadata.classDetectionCount[i] += sampleMetadata[index].classDetectionCount[i];
          }
          lmMetadata.score += sampleMetadata[index].score;
        }
        lmMetadata.score /= static_cast<float>(cluster.size());
        lmMean[0] /= static_cast<float>(cluster.size());
        lmMean[1] /= static_cast<float>(cluster.size());

        lmMeans.push_back(lmMean);
        lmMetadatas.push_back(lmMetadata);
      }

      // Calculate 2x2 position covariance matrix. We assume that there is no significan relation between class and position
      // Thus allowing us to easily extend this to 3x3, which includes class variance

      for (unsigned int i = 0; i < clusters.size(); i++)
      {
        vector<unsigned int> cluster = clusters[i];

        MatrixXf cov(2,2);

        vector<float> X;
        vector<float> Y;
        for (auto index : cluster)
        {
          X.push_back(samples[index][0]);
          Y.push_back(samples[index][1]);
        }

        cov(0, 0) = calculate_covariance(X, X);
        cov(0, 1) = calculate_covariance(X, Y);
        cov(1, 0) = cov(0, 1);
        cov(1, 1) = calculate_covariance(Y, Y);

        positionCovariances.push_back(cov);
      }
    } else {
      // Just take all the particles from the best particle
      lmMeans = bestParticle.xf();
      lmMetadatas = bestParticle.metadata();
      positionCovariances = bestParticle.Pf();
    }

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("Output averaging took %f s. That is %f s per particle", time_round, time_round / this->particles.size());

    t1 = std::chrono::steady_clock::now();

    ROS_INFO("Number of landmarks (total): %d", lmMeans.size());

    // Get the landmarks that have a high enough score
    vector<VectorXf> filteredLandmarks;
    vector<LandmarkMetadata> filteredMeta;
    vector<int> filteredLandmarkIndices;
    vector<Matrix2f> filteredCovariances;

    for (int i = 0; i < lmMeans.size(); i++)
    {
      if (lmMetadatas[i].score >= this->acceptance_score)
      {
        filteredCovariances.push_back(positionCovariances[i]);
        filteredMeta.push_back(lmMetadatas[i]);
        filteredLandmarks.push_back(lmMeans[i]);
        filteredLandmarkIndices.push_back(i);
      }
    }

    ROS_INFO("Number of actual landmarks: %d", filteredLandmarks.size());

    // Create the observation_msgs things
    ugr_msgs::ObservationWithCovarianceArrayStamped global;
    ugr_msgs::ObservationWithCovarianceArrayStamped local;
    global.header.frame_id = this->slam_world_frame;
    global.header.stamp = ros::Time::now();
    local.header.frame_id = this->base_link_frame;
    local.header.stamp = ros::Time::now();

    for (int i = 0; i < filteredLandmarks.size(); i++)
    {

      ugr_msgs::ObservationWithCovariance global_ob;
      ugr_msgs::ObservationWithCovariance local_ob;

      int observation_class = 0;
      int max_count = 0;
      int total_count = 0;
      for (int j = 0; j < LANDMARK_CLASS_COUNT; j++)
      {
        total_count += filteredMeta[i].classDetectionCount[j];
        if (filteredMeta[i].classDetectionCount[j] > max_count)
        {
          observation_class = j;
          max_count = filteredMeta[i].classDetectionCount[j];
        }
      }

      // Calculate the observation class (co)variance
      // First get the odds of a specific class p
      float p[LANDMARK_CLASS_COUNT];
      for (unsigned int j = 0; j < LANDMARK_CLASS_COUNT; j++)
      {
        p[j] = filteredMeta[i].classDetectionCount[j] / total_count;
      }

      float obsClassMean = 0.0;
      for (unsigned int j = 0; j < LANDMARK_CLASS_COUNT; j++)
      {
        obsClassMean += p[j] * j;
      }

      float obsCovariance = 0.0;
      for (unsigned int j = 0; j < LANDMARK_CLASS_COUNT; j++)
      {
        obsCovariance += pow(j - obsClassMean, 2) * p[j];
      }

      auto covarianceMatrix = boost::array<double, 9>({filteredCovariances[i](0, 0), filteredCovariances[i](0, 1), 0.0, filteredCovariances[i](1, 0), filteredCovariances[i](1, 1), 0.0, 0, 0, obsCovariance});

      global_ob.covariance = covarianceMatrix;
      local_ob.covariance = covarianceMatrix;

      global_ob.observation.observation_class = observation_class;
      local_ob.observation.observation_class = observation_class;

      float belief = max(min((1 - exp(-1 * belief_factor * filteredMeta[i].score)), 1.0), 0.0);
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

    // Odometry message (for correction of state estimation)
    // So from world_frame to base_link_frame
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

    if(this->setmap_srv_client.exists()) {
      // Initialize global request
      slam_controller::SetMap global_srv;
      global_srv.request.map = global;
      global_srv.request.name = this->globalmap_namespace;

      // Set global map with Service
      if (!this->setmap_srv_client.call(global_srv)) {
        ROS_WARN("Global map service call failed!");
      }
      
      // Initialize local request
      slam_controller::SetMap local_srv;
      local_srv.request.map = local;
      local_srv.request.name = this->localmap_namespace;

      // Set local map with Service
      if (!this->setmap_srv_client.call(local_srv)) {
        ROS_WARN("Local map service call failed!");
      }
    } else {
      ROS_WARN("SetMap service call does not exist (yet)!");
    }

    // Publish odometry
    this->odomPublisher.publish(odom);

    t2 = std::chrono::steady_clock::now();

    time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    ROS_INFO("Output publishing took %f s.", time_round);
  }
}