#include "color_connector.hpp"

namespace pathplanning {

ColorConnector::ColorConnector(ros::NodeHandle &n) : n_(n) {}

std::pair<std::vector<Node *>, std::vector<Node *>>
ColorConnector::get_color_lines(const std::vector<std::vector<double>> &cones,
                                const std_msgs::Header &header) {
  std::vector<Node *> blue_line;
  std::vector<Node *> yellow_line;
  return std::make_pair(blue_line, yellow_line);
}

} // namespace pathplanning