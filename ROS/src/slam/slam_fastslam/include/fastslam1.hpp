#ifndef FASTSLAM1_HPP
#define FASTSLAM1_HPP

#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

#include "fastslam_core.hpp"

#include <Eigen/Dense>

#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include <nav_msgs/Path.h>

#include "managed_node.hpp"
#include "node_fixture/node_fixture.hpp"

using namespace std;

namespace slam {
struct ObsOptions {
  double eps;
  double max_range;
  double max_half_fov;
  double expected_range;
  double expected_half_fov; // radians, single side
  double acceptance_score;
  double penalty_score;
  double minThreshold;
};

class FastSLAM1 : public node_fixture::ManagedNode {
public:
  explicit FastSLAM1(ros::NodeHandle &n);
  ~FastSLAM1() { this->particles.clear(); }

  static tf2_ros::TransformBroadcaster br;

  // This functions executes a FastSLAM1.0 step
  void active() override;
  void step();
  void doConfigure() override;

private:
  ObsOptions *options;
  ObsOptions lidarOptions;
  ObsOptions cameraOptions;

  // ROS
  ros::NodeHandle n;

  // ROS parameters
  string base_link_frame;
  string world_frame;
  string map_frame;
  string slam_base_link_frame;
  string lidar_frame;

  bool post_clustering;
  int particle_count;
  bool average_output_pose;
  int effective_particle_count;
  int min_clustering_point_count;
  double observe_prob;
  double clustering_eps;
  double belief_factor;

  double publish_rate;

  bool doSynchronous;

  double latestTime;

  bool firstRound;
  bool gotFirstObservations;
  bool updateRound;

  ugr_msgs::ObservationWithCovarianceArrayStamped observations;

  // Subscribers
  ros::Subscriber observationsSubscriber;

  // Publishers
  // ros::Publisher globalPublisher;
  // ros::Publisher localPublisher;
  ros::Publisher odomPublisher;
  ros::Publisher particlePosePublisher;

  // Diagnostic publisher
  std::unique_ptr<node_fixture::DiagnosticPublisher> diagPublisher;

  // Set Map Service Client
  ros::ServiceClient setmap_srv_client;
  string globalmap_namespace;
  string localmap_namespace;

  // TF2
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  message_filters::Subscriber<ugr_msgs::ObservationWithCovarianceArrayStamped>
      obs_sub;
  tf2_ros::MessageFilter<ugr_msgs::ObservationWithCovarianceArrayStamped>
      tf2_filter;

  // Handlers

  void handleObservations(
      const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs);

  void publishOutput(ros::Time);

  void predict(Particle &particle, double dDist, double dYaw, double dt);

  void build_associations(Particle &,
                          ugr_msgs::ObservationWithCovarianceArrayStamped &,
                          vector<VectorXf> &, vector<VectorXf> &, vector<int> &,
                          vector<int> &, vector<int> &, vector<float> &,
                          vector<float> &);

  double compute_particle_weight(Particle &particle, vector<VectorXf> &z,
                                 vector<int> &idf, MatrixXf &R,
                                 vector<VectorXf> &zp, vector<MatrixXf> &Hv,
                                 vector<MatrixXf> &Hf, vector<MatrixXf> &Sf);

  void landmarks_to_observations(vector<VectorXf> &lm, vector<VectorXf> &obs,
                                 VectorXf &pose);
  void landmark_to_observation(VectorXf &lm, VectorXf &obs, VectorXf &pose);

  void observations_to_landmarks(vector<VectorXf> &obs, vector<VectorXf> &lm,
                                 VectorXf &pose);
  void observation_to_landmark(VectorXf &obs, VectorXf &lm, VectorXf &pose);

  // Other stuff
  array<double, 3> prev_state; // x, y, yaw

  chrono::steady_clock::time_point prev_time;
  chrono::steady_clock::time_point prev_predict_time;
  chrono::steady_clock::time_point prev_publish_time;

  vector<Particle> particles;

  ros::Time prev_transform_time;

  // Yaw unwrapping threshold
  float yaw_unwrap_threshold;

  Eigen::MatrixXf Q;
  Eigen::MatrixXf R;
};
} // namespace slam

#endif