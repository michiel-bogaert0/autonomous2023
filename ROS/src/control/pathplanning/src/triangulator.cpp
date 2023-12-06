#include <ros/ros.h>
#include <triangulator.hpp>

namespace pathplanning {
Triangulator::Triangulator(ros::NodeHandle &n)
    : n_(n),
      triangulation_min_var_(n.param<double>("triangulation_min_var", 100.0)),
      triangulation_var_threshold_(
          n.param<double>("triangulation_var_threshold", 1.2)),
      max_iter_(n.param<int>("max_iter", 100)),
      max_angle_change_(n.param<double>("max_angle_change", 0.5)),
      max_path_distance_(n.param<double>("max_path_distance", 6.0)),
      safety_dist_(n.param<double>("safety_dist", 1.0)),
      safety_dist_squared_(safety_dist_ * safety_dist_),
      stage1_rect_width_(n.param<double>("stage1_rect_width", 1.2)),
      stage1_threshold_bad_points_(
          n.param<int>("stage1_threshold_bad_points", 2)),
      stage1_threshold_center_points_(
          n.param<int>("stage1_threshold_center_points", 3)),
      stage2_rect_width_(n.param<double>("stage2_rect_width", 1.2)),
      stage2_threshold_bad_points_(
          n.param<int>("stage2_threshold_bad_points", 2)),
      stage2_threshold_center_points_(
          n.param<int>("stage2_threshold_center_points", 3)),
      max_depth_(n.param<int>("max_depth", 5)),
      continuous_dist_(n.param<double>("continuous_dist", 4.0)),
      range_front_(n.param<double>("range_front", 20.0)),
      range_behind_(n.param<double>("range_behind", 0.0)),
      range_sides_(n.param<double>("range_sides", 3.0)),
      vis_namespace_(n.param<std::string>("vis_namespace",
                                          std::string("pathplanning_vis"))),
      vis_lifetime_(n.param<double>("vis_lifetime", 0.2)),
      debug_visualisation_(n.param<bool>("debug_visualisation", false)),
      triangulation_paths(
          max_iter_, max_angle_change_, max_path_distance_, safety_dist_,
          stage1_rect_width_, stage1_threshold_bad_points_,
          stage1_threshold_center_points_, stage2_rect_width_,
          stage2_threshold_bad_points_, stage2_threshold_center_points_,
          max_depth_, continuous_dist_) {
  this->vis_points_ = ros::Publisher();
  this->vis_lines_ = ros::Publisher();

  if (this->debug_visualisation_) {
    this->vis_points_ = n.advertise<visualization_msgs::MarkerArray>(
        "/output/debug/markers", 10);
    this->vis_lines_ =
        n.advertise<geometry_msgs::PoseArray>("/output/debug/poses", 10);
  }

  this->vis_ = this->vis_points_ && this->vis_lines_;
}

std::pair<std::vector<Node *>, std::vector<std::vector<Node *>>>
Triangulator::get_path(const std::vector<std::vector<double>> &cones,
                       const std_msgs::Header &header) {
  double range_behind = range_behind_;
  double range_sides = range_sides_;
  double range_front = range_front_;

  std::vector<std::vector<double>> filtered_cones;
  std::vector<std::vector<double>> position_cones;
  std::vector<int> cone_classes;
  for (const auto &cone : cones) {
    position_cones.push_back({cone[0], cone[1]});

    // Only keep cones within a rectangle around the car
    if (cone[0] >= -range_behind && std::abs(cone[1]) <= range_sides &&
        cone[0] <= range_front) {
      filtered_cones.push_back({cone[0], cone[1]});
      cone_classes.push_back(cone[2]);
    }
  }

  int tries = -1;
  while (filtered_cones.size() < 4) {
    tries++;
    if (tries >= 3) {
      return {};
    }
    ROS_INFO_STREAM("Not enough cones for triangulation. Trying again with a "
                    "larger rectangle");

    range_behind *= 2;
    range_sides *= 2;
    range_front *= 2;

    filtered_cones.clear();
    cone_classes.clear();
    for (const auto &cone : position_cones) {
      if (cone[0] >= -range_behind && std::abs(cone[1]) <= range_sides &&
          cone[0] <= range_front) {
        filtered_cones.push_back(cone);
        cone_classes.push_back(cone[2]);
      }
    }
  }

  // Perform triangulation and get the (useful) center points
  auto result_center_points = get_center_points(
      filtered_cones, cone_classes, this->triangulation_min_var_,
      this->triangulation_var_threshold_, this->range_front_);
  //   auto triangulation_centers = std::get<0>(result_center_points);
  auto center_points = std::get<1>(result_center_points);
  auto bad_points = std::get<2>(result_center_points);

  // Publish visualisation topics if needed
  if (this->vis_) {
    this->publish_points(center_points, header,
                         this->vis_namespace_ + "/center_points", {0, 1, 0, 1},
                         0.2);
    this->publish_points(bad_points, header,
                         this->vis_namespace_ + "/bad_points", {1, 0, 0, 1},
                         0.2);
  }

  //   Node *root_node;
  std::vector<Node *> leaves;
  std::tuple<Node *, std::vector<Node *>> all_paths =
      this->triangulation_paths.get_all_paths(bad_points, center_points,
                                              position_cones, range_front_);
  //   root_node = std::get<0>(all_paths);
  leaves = std::get<1>(all_paths);

  if (leaves.empty()) {
    return {};
  }

  return get_best_path(leaves, cones);
}

std::pair<std::vector<Node *>, std::vector<std::vector<Node *>>>
Triangulator::get_best_path(const std::vector<Node *> &leaves,
                            const std::vector<std::vector<double>> &cones) {

  std::vector<std::array<double, 2>> costs;
  std::vector<std::vector<Node *>> paths;
  std::vector<size_t> path_lengths(1, 0);

  // Iterate each leaf
  for (Node *leaf : leaves) {
    // Find the path connecting this leaf to the root and reverse it
    std::vector<Node *> path;
    Node *parent = leaf;
    if (leaf != nullptr) {
      while (parent->parent != nullptr) {
        path.push_back(parent);
        parent = parent->parent;
      }
      // the last node points back to the root node [0, 0] which is not included
      // in the path
      std::reverse(path.begin(), path.end());
    }

    if (!path.empty()) {
      // Calculate the path cost
      std::tuple<double, double> costs_tuple =
          this->triangulation_paths.get_cost_branch(path, cones,
                                                    this->range_front_);
      double angle_cost = std::get<0>(costs_tuple);
      double length_cost = std::get<1>(costs_tuple);
      costs.push_back({angle_cost, length_cost});
      paths.push_back(path);
      path_lengths.push_back(path.size());
    }
  }

  if (paths.empty()) {
    return {};
  }

  // Normalize costs
  double max_angle_cost = std::max_element(costs.begin(), costs.end(),
                                           [](const std::array<double, 2> &a,
                                              const std::array<double, 2> &b) {
                                             return a[0] < b[0];
                                           })
                              ->at(0);
  double max_length_cost = std::max_element(costs.begin(), costs.end(),
                                            [](const std::array<double, 2> &a,
                                               const std::array<double, 2> &b) {
                                              return a[1] < b[1];
                                            })
                               ->at(1);
  for (auto &cost : costs) {
    cost[0] /= max_angle_cost;
    cost[1] /= max_length_cost;
  }

  // Calculate total cost for each path
  std::vector<double> total_cost;
  for (const auto &cost : costs) {
    total_cost.push_back(cost[0] + cost[1]);
  }

  std::vector<std::vector<Node *>> sorted_paths;
  if (this->debug_visualisation_) {
    // order the paths by their total cost low to high
    std::vector<size_t> indices(total_cost.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&total_cost](size_t i1, size_t i2) {
                return total_cost[i1] < total_cost[i2];
              });

    // Sort paths by their total cost
    for (const auto &index : indices) {
      sorted_paths.push_back(paths[index]);
    }
  }

  // Get the index of the least-cost path
  auto idx = std::min_element(total_cost.begin(), total_cost.end());
  size_t index = std::distance(total_cost.begin(), idx);

  std::pair<std::vector<pathplanning::Node *>,
            std::vector<std::vector<pathplanning::Node *>>>
      path_pair;
  path_pair.first = paths[index];  // First element: array of nodes (best path)
  path_pair.second = sorted_paths; // Second element: array of arrays of nodes
                                   // (all paths sorted by cost)

  return path_pair;
}

void Triangulator::publish_points(
    const std::vector<std::vector<double>> &points,
    const std_msgs::Header &header, const std::string &vis_namespace,
    const std::array<double, 4> &color, double scale = 0.1) {
  visualization_msgs::MarkerArray marker_array;

  for (size_t i = 0; i < points.size(); i++) {
    const auto &point = points[i];
    visualization_msgs::Marker marker;

    marker.header = header;
    marker.ns = vis_namespace;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.id = i;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = point[0];
    marker.pose.position.y = point[1];

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    marker.lifetime = ros::Duration(this->vis_lifetime_);

    marker_array.markers.push_back(marker);
  }

  this->vis_points_.publish(marker_array);
}

void Triangulator::publish_line(const std::vector<std::vector<double>> &line,
                                const std_msgs::Header &header) {
  std::vector<geometry_msgs::Pose> poses;

  for (size_t i = 0; i < line.size(); i++) {
    const auto &point = line[i];
    geometry_msgs::Pose pose;
    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;

    // Fill position from path point
    position.x = point[0];
    position.y = point[1];
    position.z = 0;

    // Fill orientation (to invalidate)
    orientation.x = 0;
    orientation.y = 0;
    orientation.z = 0;
    orientation.w = 0;

    // Fill pose and add to array
    pose.position = position;
    pose.orientation = orientation;
    poses.push_back(pose);
  }

  geometry_msgs::PoseArray output;
  output.header = header;
  output.poses = poses;

  this->vis_lines_.publish(output);
}

} // namespace pathplanning