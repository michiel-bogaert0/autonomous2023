#ifndef MCL_HPP
#define MCL_HPP

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

#include "particle.hpp"

#include <Eigen/Dense>

#include "kdtreepoint.hpp"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include <nav_msgs/Odometry.h>

#include <ugr_msgs/ObservationWithCovariance.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

#include "managed_node.hpp"
#include "node_fixture/node_fixture.hpp"

using namespace std;

#define LANDMARK_CLASS_COUNT 4

namespace slam {

// Commented out because it is not used (linting)
// struct LandmarkSearchResult {
//   int index;
//   double distance;
// };

struct Landmark {
  // int landmarkClass; (linting)
  MatrixXf variance;
  VectorXf pose;
};

class MCL : public node_fixture::ManagedNode {
public:
  explicit MCL(ros::NodeHandle &n);
  ~MCL() { this->particles.clear(); }

  static tf2_ros::TransformBroadcaster br;

  void step();
  void doActivate() override;
  void active() override;

private:
  // ROS
  ros::NodeHandle n;

  // ROS parameters
  string base_link_frame;
  string world_frame;
  string map_frame;
  string slam_base_link_frame;

  int particle_count;
  int effective_particle_count;

  double observe_dt;
  double eps;

  // Internal parameters
  ugr_msgs::ObservationWithCovarianceArrayStamped observations;
  double latestTime;

  double max_range;
  double max_half_fov;

  vector<Landmark> _map;
  kdt::KDTree<KDTreePoint> _trees[LANDMARK_CLASS_COUNT];

  array<double, 3> prev_state; // x, y, yaw

  chrono::steady_clock::time_point prev_time;

  vector<Particle> particles;

  // Yaw unwrapping threshold
  float yaw_unwrap_threshold;

  Eigen::MatrixXf Q;
  Eigen::MatrixXf R;

  // Subscribers
  ros::Subscriber observationsSubscriber;
  ros::Subscriber mapSubscriber;

  // Publishers
  ros::Publisher odomPublisher;
  ros::Publisher particlePosePublisher;

  // Diagnostic Publisher
  std::unique_ptr<node_fixture::DiagnosticPublisher> diagPublisher;

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
  void
  handleMap(const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs);

  void motion_update(Particle &particle, double dDist, double dYaw);

  void publishOutput(ros::Time lookupTime);

  void resample_particles();

  void predict(Particle &particle, double dDist, double dYaw);
  double sensor_update(Particle &particle, vector<VectorXf> &z,
                       vector<int> &associations);

  void build_associations(Particle &,
                          ugr_msgs::ObservationWithCovarianceArrayStamped &,
                          vector<VectorXf> &, vector<int> &);

  void landmarks_to_observations(vector<VectorXf> &lm, vector<VectorXf> &obs,
                                 VectorXf &pose);
  void landmark_to_observation(VectorXf &lm, VectorXf &obs, VectorXf &pose);

  void observations_to_landmarks(vector<VectorXf> &obs, vector<VectorXf> &lm,
                                 VectorXf &pose);
  void observation_to_landmark(VectorXf &obs, VectorXf &lm, VectorXf &pose);
};
} // namespace slam

#endif