#include <paths.hpp>
#include <ros/ros.h>

namespace pathplanning {
std::pair<Node *, std::vector<Node *>> TriangulationPaths::get_all_paths(
    const std::vector<std::vector<double>> &bad_points,
    const std::vector<std::vector<double>> &center_points,
    const std::vector<std::vector<double>> &cones) {
  Node *root_node = new Node(0, 0, 0, nullptr, std::vector<Node *>(), 0, 0);

  // Queue contains node to search, path already searched (corresponding to that
  // node), current depth
  std::vector<std::tuple<Node *, std::vector<std::array<double, 2>>, int>>
      queue({std::make_tuple(root_node,
                             std::vector<std::array<double, 2>>(
                                 {std::array<double, 2>({0.0, 0.0})}),
                             0)});

  std::vector<Node *> leaves;
  int iteration = 0;

  while (!queue.empty()) {
    iteration++;

    if (iteration >= this->max_iter)
      break;

    // Get the next element from the queue
    auto t = queue[0];
    auto parent = std::get<0>(t);
    auto path = std::get<1>(t);
    auto depth = std::get<2>(t);

    queue.erase(queue.begin());

    bool first_pose_found = true;
    if (path.size() <= 1) {
      first_pose_found = false;
    }

    bool child_found = false;

    // Handle first pose
    if (!first_pose_found) {
      bool second_child_found = false;

      // Search all nodes in continuous distance and add node with smallest
      // angle_change from car orientation
      double min_angle_change = 1000;
      Node *node = nullptr;
      Node *node2 = nullptr;

      // Get the center points to this element that are within close
      // distance and sort them by angle_change
      std::vector<PointInfo> next_nodes;
      next_nodes = sort_by_angle_change(center_points, {parent->x, parent->y},
                                        parent->angle, this->max_angle_change,
                                        this->continuous_dist_);

      for (PointInfo next_node : next_nodes) {
        if (pathplanning::check_if_feasible_child(
                *parent, path, next_node.point, bad_points, center_points,
                cones, this->max_angle_change, this->safety_dist_squared, 0,
                this->stage1_threshold_bad_points_,
                this->stage1_threshold_center_points_)) {

          if (!child_found) {
            node = new Node(next_node.point[0], next_node.point[1],
                            next_node.distance_squared, parent,
                            std::vector<Node *>(), next_node.angle,
                            next_node.angle_change);
            child_found = true;
          } else {
            if (!second_child_found) {
              if (node->x == next_node.point[0] &&
                  node->y == next_node.point[1]) {
                continue;
              }
              node2 = new Node(next_node.point[0], next_node.point[1],
                               next_node.distance_squared, parent,
                               std::vector<Node *>(), next_node.angle,
                               next_node.angle_change);
              second_child_found = true;
              break;
            }
          }
        }
      }
      if (child_found) {
        if (second_child_found) {
          std::vector<std::array<double, 2>> new_path(path);
          new_path.push_back({node2->x, node2->y});
          parent->children.push_back(node2);

          queue.push_back(std::make_tuple(node2, new_path, depth + 1));
        }

        path.push_back({node->x, node->y});
        parent->children.push_back(node);
        parent = node;
      }
    }

    // Get the closest center points to this element that are within close
    // distance (stage 1)
    std::vector<std::vector<double>> next_nodes;
    next_nodes = sort_closest_to(center_points, {parent->x, parent->y},
                                 this->continuous_dist_);

    // if first point of path close to last point of path, stop loop
    if (path.size() > 20 &&
        distance_squared(path[1][0], path[1][1], next_nodes[0][0],
                         next_nodes[0][1]) < this->close_path_dist) {
      leaves.push_back(parent);
      break;
    }

    child_found = false;

    float max_dist = pow(this->continuous_dist_, 2);

    for (auto next_pos : next_nodes) {
      if (pathplanning::check_if_feasible_child(
              *parent, path, next_pos, bad_points, center_points, cones,
              this->stage1_max_angle_change_, this->safety_dist_squared,
              this->stage1_rect_width_, this->stage1_threshold_bad_points_,
              this->stage1_threshold_center_points_)) {

        double distance_node =
            pow(parent->x - next_pos[0], 2) + pow(parent->y - next_pos[1], 2);
        double angle_node =
            atan2(next_pos[1] - parent->y, next_pos[0] - parent->x);
        double angle_change = angle_node - parent->angle;

        Node *node = new Node(next_pos[0], next_pos[1], distance_node, parent,
                              std::vector<Node *>(), angle_node, angle_change);

        if (distance_node > max_dist) {
          break;
        }

        std::vector<std::array<double, 2>> new_path(path);
        new_path.push_back({node->x, node->y});
        parent->children.push_back(node);

        queue.push_back(std::make_tuple(node, new_path, depth + 1));

        if (!child_found) {
          max_dist = distance_node + this->stage1_dist_window_;
          child_found = true;
        }
      }
    }

    if (child_found) {
      continue;
    }

    // Now we are at the second stage. We basically look for a node in the
    // distance (to bridge a gap) and add it to the stack to explore further We
    // also limit the depth of this stage (stage 2)
    if (depth >= this->max_depth_) {
      leaves.push_back(parent);
      continue;
    }

    // Get the closest center points to this element that are within
    // stage2_max_dist
    std::vector<std::vector<double>> next_nodes2;
    next_nodes2 = sort_closest_to(center_points, {parent->x, parent->y},
                                  this->stage2_max_dist_);
    child_found = false;

    for (auto next_pos : next_nodes2) {
      if (pathplanning::check_if_feasible_child(
              *parent, path, next_pos, bad_points, center_points, cones,
              this->max_angle_change, this->safety_dist_squared,
              this->stage2_rect_width_, this->stage2_threshold_bad_points_,
              this->stage2_threshold_center_points_)) {

        double distance_node =
            pow(parent->x - next_pos[0], 2) + pow(parent->y - next_pos[1], 2);
        double angle_node =
            atan2(next_pos[1] - parent->y, next_pos[0] - parent->x);
        double angle_change = angle_node - parent->angle;

        // The first pose of the path is an exeption and its angle should be
        // oriented in the same direction as the car
        if (!first_pose_found) {
          angle_node = 0;
        }

        Node *node = new Node(next_pos[0], next_pos[1], distance_node, parent,
                              std::vector<Node *>(), angle_node, angle_change);

        std::vector<std::array<double, 2>> new_path(path);
        new_path.push_back({node->x, node->y});
        parent->children.push_back(node);

        queue.push_back(std::make_tuple(node, new_path, depth + 1));
      }
    }

    if (!child_found) {
      leaves.push_back(parent);
    }
  }

  return std::make_pair(root_node, leaves);
}

std::pair<double, double> TriangulationPaths::get_cost_branch(
    const std::vector<Node *> &branch,
    const std::vector<std::vector<double>> &cones) {

  std::vector<double> angle_changes;
  std::vector<double> node_distances;
  for (const Node *point : branch) {
    angle_changes.push_back(std::abs(point->angle_change));
    node_distances.push_back(point->distance);
  }

  // angle should not change much over one path
  double angle_cost = calculate_variance(angle_changes);

  // longer paths usually work better as they make use of more center points
  // having many segments along a path is usually a good path
  double distance =
      std::accumulate(node_distances.begin(), node_distances.end(), 0.0);
  double length_cost = 1 / distance + 10.0 / node_distances.size();

  double penalty = 0;

  // Iterate over the path, get all cones in a certain distance to line, and
  // apply a penalty every time a cone is on the wrong side
  for (const auto &point : branch) {
    std::vector<double> line_distances_squared(cones.size());
    std::vector<double> distances_squared(cones.size());
    std::vector<std::vector<double>> close_cones;
    std::vector<double> diff_angles;
    std::vector<int> close_classes;

    for (size_t i = 0; i < cones.size(); ++i) {
      line_distances_squared[i] =
          std::cos(point->angle) * (cones[i][1] - point->y) -
          std::sin(point->angle) * (cones[i][0] - point->x);
      distances_squared[i] =
          distance_squared(point->x, point->y, cones[i][0], cones[i][1]);

      if (line_distances_squared[i] < 5 * 3 && distances_squared[i] < 5 * 3) {
        close_cones.push_back(cones[i]);
        diff_angles.push_back(
            std::atan2(cones[i][1] - point->y, cones[i][0] - point->x) -
            point->angle);
        if (diff_angles.back() > M_PI) {
          diff_angles.back() -= 2 * M_PI;
        } else if (diff_angles.back() < -M_PI) {
          diff_angles.back() += 2 * M_PI;
        }
        close_classes.push_back(cones[i][2]);
      }
    }

    double penalty_amount = 0;
    for (size_t i = 0; i < close_cones.size(); ++i) {
      if (diff_angles[i] > 0.0 && close_classes[i] == 1) {
        penalty_amount++;
      } else if (diff_angles[i] < 0.0 && close_classes[i] == 0) {
        penalty_amount++;
      }
    }

    penalty += penalty_amount / 100;
  }

  return std::make_pair(angle_cost + penalty, length_cost + penalty);
}

} // namespace pathplanning