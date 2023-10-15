#include "pathplanning.hpp"

namespace pathplanning {

TransformFrames::TransformFrames(ros::NodeHandle &n)
    : nh(n), tfBuffer(), tfListener(tfBuffer) {}

Pathplanning::Pathplanning(
    ros::NodeHandle &n, bool debug_visualisation, std::string vis_namespace,
    double vis_lifetime, int max_iter, double max_angle_change,
    double safety_dist, double triangulation_min_var,
    double triangulation_var_threshold, double max_path_distance,
    double range_front, double range_behind, double range_sides,
    ros::Publisher vis_points, ros::Publisher vis_lines,
    double stage1_rect_width_, int stage1_threshold_bad_points_,
    int stage1_threshold_center_points_, double stage2_rect_width_,
    int stage2_threshold_bad_points_, int stage2_threshold_center_points_,
    int max_depth_, double continuous_dist_,
    double min_distance_away_from_start, double max_distance_away_from_start)
    : n_(n), frametf_(n), debug_visualisation_(debug_visualisation),
      vis_namespace_(vis_namespace), vis_lifetime_(vis_lifetime),
      max_iter_(max_iter), max_angle_change_(max_angle_change),
      safety_dist_(safety_dist), triangulation_min_var_(triangulation_min_var),
      triangulation_var_threshold_(triangulation_var_threshold),
      max_path_distance_(max_path_distance), range_front_(range_front),
      range_behind_(range_behind), range_sides_(range_sides),
      vis_points_(vis_points), vis_lines_(vis_lines),
      min_distance_away_from_start(min_distance_away_from_start),
      max_distance_away_from_start(max_distance_away_from_start),
      triangulator_(
          n, triangulation_min_var, triangulation_var_threshold, max_iter,
          max_angle_change, max_path_distance, safety_dist, range_front,
          range_behind, range_sides, vis_points, vis_lines, vis_namespace,
          vis_lifetime, stage1_rect_width_, stage1_threshold_bad_points_,
          stage1_threshold_center_points_, stage2_rect_width_,
          stage2_threshold_bad_points_, stage2_threshold_center_points_,
          max_depth_, continuous_dist_) {

  this->path_pub_ = n_.advertise<nav_msgs::Path>("/output/path", 10);
  this->map_sub_ = n_.subscribe("/input/local_map", 10,
                                &Pathplanning::receive_new_map, this);
}

void Pathplanning::receive_new_map(
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

void Pathplanning::compute(const std::vector<std::vector<double>> &cones,
                           const std_msgs::Header &header) {
  std::vector<Node *> path = this->triangulator_.get_path(cones, header);

  if (path.empty()) {
    ROS_INFO_STREAM("No path found");
    return;
  }

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

  bool away_from_start = false;

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

    double distance = pow(node->x, 2) + pow(node->y, 2);

    if (away_from_start && distance < max_distance_away_from_start) {
      // Close loop
      poses.push_back(zero_pose);
      break;
    }
    if (!away_from_start && distance > min_distance_away_from_start) {
      away_from_start = true;
    }
  }

  nav_msgs::Path output;
  output.header.frame_id = header.frame_id;
  output.poses = poses;
  output.header.stamp = header.stamp;

  nav_msgs::Path output_transformed = this->frametf_.pose_transform(output);

  this->path_pub_.publish(output_transformed);
}

} // namespace pathplanning