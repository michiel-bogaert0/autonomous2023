#ifndef FASTSLAM1_HPP
#define FASTSLAM1_HPP

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ugr_msgs/Observations.h>

#include "fastslam_core.hpp"

#include <Eigen/Dense>

#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std;

namespace slam
{
  class FastSLAM1
  {
    public: 
      FastSLAM1(ros::NodeHandle &n);
      ~FastSLAM1() {
        this->particles.clear();
      }

      static tf2_ros::TransformBroadcaster br;

    private:
      
      // ROS
      ros::NodeHandle n;

      // ROS parameters
      string base_link_frame;
      string world_frame;
      string slam_world_frame;
      int particle_count;
      double penalty_score;
      int effective_particle_count;
      double observe_dt;
      double eps;

      double latestTime;
      
      double minThreshold;
      double acceptance_score;

      double max_range;
      double max_half_fov;
      double expected_range;
      double expected_half_fov; // radians, single side 

      // Subscribers
      ros::Subscriber observationsSubscriber;
      
      // Publishers
      ros::Publisher globalPublisher;
      ros::Publisher localPublisher;
      ros::Publisher odomPublisher;

      // TF2
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener;
      message_filters::Subscriber<ugr_msgs::Observations> obs_sub;
      tf2_ros::MessageFilter<ugr_msgs::Observations> tf2_filter;

      // Handlers

      void handleObservations(const ugr_msgs::ObservationsConstPtr &obs);

      void publishOutput();

      void predict(Particle &particle, double dDist, double dYaw);

      void build_associations(Particle &, ugr_msgs::Observations &, vector<VectorXf> &, vector<VectorXf> &, vector<int> &, vector<int>&, vector<int>&);

      double compute_particle_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

      void landmarks_to_observations(vector<VectorXf> &lm, vector<VectorXf> &obs, VectorXf &pose);
      void landmark_to_observation(VectorXf &lm, VectorXf &obs, VectorXf &pose);

      void observations_to_landmarks(vector<VectorXf> &obs, vector<VectorXf> &lm, VectorXf &pose);
      void observation_to_landmark(VectorXf &obs, VectorXf & lm, VectorXf &pose);

      // Other stuff
      array<double, 3> prev_state; // x, y, yaw

      chrono::steady_clock::time_point  prev_time;

      vector<Particle> particles; 

      Eigen::MatrixXf Q;
      Eigen::MatrixXf R;
  };
}

#endif