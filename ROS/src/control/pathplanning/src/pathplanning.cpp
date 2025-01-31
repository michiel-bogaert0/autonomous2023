#include "pathplanning.hpp"

namespace pathplanning {

TransformFrames::TransformFrames(ros::NodeHandle &n)
    : nh(n), tfBuffer(), tfListener(tfBuffer) {}

Pathplanning::Pathplanning(ros::NodeHandle &n)
    : ManagedNode(n, "pathplanning"), n_(n), frametf_(n), triangulator_(n),
      debug_visualisation_(n.param<bool>("debug_visualisation", false)),
      use_orange_cones_(n.param<bool>("use_orange_cones", false)) {}

void Pathplanning::doConfigure() {}
void Pathplanning::doActivate() {
  this->path_pub_ = n_.advertise<ugr_msgs::PathWithIds>("/output/path", 10);
  this->map_sub_ = n_.subscribe("/input/local_map", 10,
                                &Pathplanning::receive_new_map, this);
  this->vis_paths_ =
      n_.advertise<ugr_msgs::PathArray>("/output/debug/all_poses", 10);
  this->diagnostics_pub = std::unique_ptr<node_fixture::DiagnosticPublisher>(
      new node_fixture::DiagnosticPublisher(n_, "CTRL PATH"));
}

void Pathplanning::receive_new_map(
    const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr &track) {
  if (!this->isActive()) {
    return;
  }
  std::vector<std::vector<double>> cones;
  for (const auto &obs_with_cov : track->observations) {
    if (!(!this->use_orange_cones_ &&
          obs_with_cov.observation.observation_class == 2)) {
      std::vector<double> cone;
      cone.push_back(obs_with_cov.observation.location.x);
      cone.push_back(obs_with_cov.observation.location.y);
      cone.push_back(obs_with_cov.observation.observation_class);
      cone.push_back(obs_with_cov.observation.id);
      cones.push_back(cone);
    }
  }

  compute(cones, track->header);
}

void Pathplanning::compute(const std::vector<std::vector<double>> &cones,
                           const std_msgs::Header &header) {
  std::pair<std::vector<Node *>, std::vector<std::vector<Node *>>> pathPair =
      this->triangulator_.get_path(cones, header);
  std::vector<Node *> path = std::get<0>(pathPair);
  std::vector<std::vector<Node *>> debugPaths = std::get<1>(pathPair);

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

  // Find ID's of the cones connected to find the center point
  std::vector<std::vector<int>> cone_ids;

  for (auto &node : path) {
    for (size_t i = 0; i < cones.size(); i++) {
      for (size_t j = 0; j < cones.size(); j++) {
        // Centerpoint
        if (i != j &&
            (fabs(((cones[i][0] + cones[j][0]) / 2 - node->x)) < 0.00001 &&
             fabs(((cones[i][1] + cones[j][1]) / 2 - node->y)) < 0.00001)) {
          cone_ids.push_back({cones[i][3], cones[j][3]});
        }
      }
    }
  }

  std::vector<ugr_msgs::PoseStampedWithIds> poses;

  for (size_t i = 0; i < path.size(); i++) {
    auto node = path[i];
    ugr_msgs::PoseStampedWithIds pose;

    pose.pose.position.x = node->x;
    pose.pose.position.y = node->y;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    pose.left_id = cone_ids[i][0];
    pose.right_id = cone_ids[i][1];

    pose.header.frame_id = header.frame_id;
    pose.header.stamp = header.stamp;

    poses.push_back(pose);
  }

  ugr_msgs::PathWithIds output;
  output.header.frame_id = header.frame_id;
  output.poses = poses;
  output.header.stamp = header.stamp;

  ugr_msgs::PathWithIds output_transformed = output;

  this->path_pub_.publish(output_transformed);

  if (this->debug_visualisation_) {
    // Publish all paths
    std::vector<nav_msgs::Path> allPaths;
    for (const auto &debugPath : debugPaths) {
      std::vector<geometry_msgs::PoseStamped> debugPoses;

      for (const auto &debug_node : debugPath) {
        geometry_msgs::PoseStamped pose;

        pose.pose.position.x = debug_node->x;
        pose.pose.position.y = debug_node->y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        pose.header.frame_id = header.frame_id;
        pose.header.stamp = header.stamp;

        debugPoses.push_back(pose);
      }

      nav_msgs::Path debug_output;
      debug_output.header.frame_id = header.frame_id;
      debug_output.poses = debugPoses;
      debug_output.header.stamp = header.stamp;

      nav_msgs::Path debug_output_transformed = debug_output;

      allPaths.push_back(debug_output_transformed);
    }
    ugr_msgs::PathArray pathArray;
    pathArray.paths = allPaths;
    vis_paths_.publish(pathArray);
  }
}

} // namespace pathplanning