#include "color_connector.hpp"

namespace pathplanning {

ColorConnector::ColorConnector(ros::NodeHandle &n) : n_(n) {}

std::pair<std::vector<Node *>, std::vector<Node *>>
ColorConnector::get_color_lines(const std::vector<std::vector<double>> &cones,
                                const std_msgs::Header &header) {

  // maximum distance between two cones in meter squared
  double max_dist = pow(6, 2);

  std::vector<Node *> blue_line;
  std::vector<Node *> yellow_line;

  // get all blue cones
  std::vector<std::vector<double>> blue_cones;
  for (const auto &cone : cones) {
    if (cone[2] == 0) {
      blue_cones.push_back(cone);
    }
  }
  // get all yellow cones
  std::vector<std::vector<double>> yellow_cones;
  for (const auto &cone : cones) {
    if (cone[2] == 1) {
      yellow_cones.push_back(cone);
    }
  }

  // node at position of the car, distance 0, angle 0
  // children NOT IMPLEMENTED
  Node *root_node = new Node(0, 0, 0, nullptr, std::vector<Node *>(), 0, 0);

  // create blue line
  blue_line.push_back(root_node);
  // get closest blue cone to the car
  while (true) {
    Node *next_cone = get_next_node(blue_cones, blue_line, max_dist);
    if (next_cone == nullptr) {
      break;
    }
    blue_line.push_back(next_cone);
  }

  // create yellow line
  yellow_line.push_back(root_node);
  // get closest yellow cone to the car
  while (true) {
    Node *next_cone = get_next_node(yellow_cones, yellow_line, max_dist);
    if (next_cone == nullptr) {
      break;
    }
    yellow_line.push_back(next_cone);
  }

  return std::make_pair(blue_line, yellow_line);
}

Node *
ColorConnector::get_next_node(const std::vector<std::vector<double>> &cones,
                              const std::vector<Node *> &line,
                              double max_dist) {
  Node *best_node = nullptr;
  double smallest_angle_change = M_PI;
  double angle_change;
  double angle;
  for (const auto &cone : cones) {
    bool isInLine =
        std::any_of(line.begin(), line.end(), [&cone](Node *node) -> bool {
          return node->x == cone[0] && node->y == cone[1];
        });
    if (!isInLine) {
      double dist =
          distance_squared(line.back()->x, line.back()->y, cone[0], cone[1]);
      if (dist < max_dist) {
        angle = angle_between(line.back()->x, line.back()->y, cone[0], cone[1]);
        angle_change = angle_difference(angle, line.back()->angle);
        if (angle_change < smallest_angle_change) {
          smallest_angle_change = angle_change;
          best_node = new Node(cone[0], cone[1], dist, line.back(),
                               std::vector<Node *>(), angle, angle_change);
        }
      }
    }
  }
  return best_node;
}

} // namespace pathplanning
