#include "boundary_estimation.hpp"

namespace pathplanning {

BoundaryEstimation::BoundaryEstimation(ros::NodeHandle &n)
    : n_(n), frametf_(n), color_connector_(n) {
  this->boundary_pub_ =
      n_.advertise<ugr_msgs::Boundaries>("/output/boundaries", 10);
  this->left_boundary_pub_ =
      n_.advertise<nav_msgs::Path>("/output/debug/left_boundary", 10);
  this->right_boundary_pub_ =
      n_.advertise<nav_msgs::Path>("/output/debug/right_boundary", 10);
  this->map_sub_ = n_.subscribe("/input/local_map", 10,
                                &BoundaryEstimation::receive_new_map, this);
  this->diagnostics_pub = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n, "CTRL BOUNDARY"));
}

void BoundaryEstimation::receive_new_map(
    const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr &track) {
  std::vector<std::vector<double>> cones;
  for (const auto &obs_with_cov : track->observations) {
    std::vector<double> cone;
    cone.push_back(obs_with_cov.observation.location.x);
    cone.push_back(obs_with_cov.observation.location.y);
    cone.push_back(obs_with_cov.observation.observation_class);
    cones.push_back(cone);
  }

  compute(cones, track->header);
}

void BoundaryEstimation::compute(const std::vector<std::vector<double>> &cones,
                                 const std_msgs::Header &header) {
  std::pair<std::vector<Node *>, std::vector<Node *>> boundaries =
      this->color_connector_.get_color_lines(cones, header);

  std::vector<geometry_msgs::PoseStamped> blue_poses;
  for (const auto &node : boundaries.first) {
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = node->x;
    pose.pose.position.y = node->y;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    pose.header.frame_id = header.frame_id;
    pose.header.stamp = header.stamp;

    blue_poses.push_back(pose);
  }
  std::vector<geometry_msgs::PoseStamped> yellow_poses;
  for (const auto &node : boundaries.second) {
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = node->x;
    pose.pose.position.y = node->y;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    pose.header.frame_id = header.frame_id;
    pose.header.stamp = header.stamp;

    yellow_poses.push_back(pose);
  }

  nav_msgs::Path left;
  left.header.frame_id = header.frame_id;
  left.poses = blue_poses;
  left.header.stamp = header.stamp;

  nav_msgs::Path right;
  right.header.frame_id = header.frame_id;
  right.poses = yellow_poses;
  right.header.stamp = header.stamp;

  ugr_msgs::Boundaries boundaries_msg;
  boundaries_msg.left_boundary = left;
  boundaries_msg.right_boundary = right;

  this->boundary_pub_.publish(boundaries_msg);
  this->left_boundary_pub_.publish(left);
  this->right_boundary_pub_.publish(right);
}

} // namespace pathplanning