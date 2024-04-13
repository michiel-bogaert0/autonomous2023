#ifndef GraphSLAM_HPP
#define GraphSLAM_HPP

#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

#include <Eigen/Dense>

#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include <nav_msgs/Odometry.h>

#include "managed_node.hpp"
#include "node_fixture/node_fixture.hpp"

#include "g2o/core/sparse_optimizer.h"

using namespace std;

namespace slam {

class GraphSLAM : public node_fixture::ManagedNode {
public:
  explicit GraphSLAM(ros::NodeHandle &n);
  ~GraphSLAM() {}

  static tf2_ros::TransformBroadcaster br;

  void step();
  void doConfigure() override;
  void active() override;

private:
  // ROS
  ros::NodeHandle n;

  // ROS parameters
  string base_link_frame;
  string world_frame;
  string map_frame;
  string slam_base_link_frame;
  string lidar_frame;

  // GraphSLAM parameters
  double publish_rate;
  bool doSynchronous;
  bool debug;

  int max_iterations;
  double association_threshold;
  double max_range;
  double max_half_angle;
  int penalty_threshold;
  Eigen::Matrix3d information_pose;
  Eigen::Matrix2d information_landmark;

  // GraphSLAM variables
  double latestTime;
  bool gotFirstObservations;
  array<double, 3> prev_state; // x, y, yaw
  int vertexCounter;
  int prevPoseIndex;
  ugr_msgs::ObservationWithCovarianceArrayStamped observations;
  g2o::SparseOptimizer optimizer;

  // Set Map Service Client
  ros::ServiceClient setmap_srv_client;
  string globalmap_namespace;
  string localmap_namespace;

  // Subscribers
  ros::Subscriber odomSubscriber;

  // Publishers
  ros::Publisher odomPublisher;
  ros::Publisher posesPublisher;
  ros::Publisher edgePosesPublisher;
  ros::Publisher edgeLandmarksPublisher;
  // Diagnostic publisher
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

  void publishOutput(ros::Time);

  chrono::steady_clock::time_point prev_publish_time;
};
} // namespace slam

#endif // GraphSLAM_HPP
