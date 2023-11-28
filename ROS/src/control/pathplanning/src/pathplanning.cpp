#include "pathplanning.hpp"

namespace pathplanning {

TransformFrames::TransformFrames(ros::NodeHandle &n)
    : nh(n), tfBuffer(), tfListener(tfBuffer) {}

Pathplanning::Pathplanning(ros::NodeHandle &n)
    : ManagedNode(n, "pathplanning"), n_(n), frametf_(n), triangulator_(n) {

  this->path_pub_ = n_.advertise<nav_msgs::Path>("/output/path", 10);
  this->map_sub_ = n_.subscribe("/input/local_map", 10,
                                &Pathplanning::receive_new_map, this);
  this->diagnostics_pub = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n, "CTRL PATH"));
}
void Pathplanning::doConfigure() {
  // add stuff that needs to be configured after selecting a (new) mission
}
void Pathplanning::receive_new_map(
    const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr &track) {
  // if (!this->isActive()){
  //   return;
  // }
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

void Pathplanning::compute(const std::vector<std::vector<double>> &cones,
                           const std_msgs::Header &header) {
  std::vector<Node *> path = this->triangulator_.get_path(cones, header);

  if (path.empty()) {
    ROS_INFO_STREAM("No path found");
    this->diagnostics_pub->publishDiagnostic(
        node_fixture::DiagnosticStatusEnum::WARN, "Path Compute Status",
        "No path found!");
    return;
  }

  this->diagnostics_pub->publishDiagnostic(
      node_fixture::DiagnosticStatusEnum::OK, "Path Compute Status",
      "Path found.");

  std::vector<geometry_msgs::PoseStamped> poses;

  // Manually add zero_pose
  geometry_msgs::PoseStamped zero_pose;
  zero_pose.pose.position.x = 0;
  zero_pose.pose.position.y = 0;
  zero_pose.pose.position.z = 0.0;

  zero_pose.pose.orientation.x = 0.0;
  zero_pose.pose.orientation.y = 0.0;
  zero_pose.pose.orientation.z = 0.0;
  zero_pose.pose.orientation.w = 1.0;

  zero_pose.header.frame_id = header.frame_id;
  zero_pose.header.stamp = header.stamp;

  poses.push_back(zero_pose);

  for (const auto &node : path) {
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

    poses.push_back(pose);
  }

  nav_msgs::Path output;
  output.header.frame_id = header.frame_id;
  output.poses = poses;
  output.header.stamp = header.stamp;

  nav_msgs::Path output_transformed = output;

  this->path_pub_.publish(output_transformed);
}

} // namespace pathplanning