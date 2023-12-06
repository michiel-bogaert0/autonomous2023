#ifndef COLOR_CONNECTOR_HPP
#define COLOR_CONNECTOR_HPP

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <utils.hpp>

namespace pathplanning {

class ColorConnector {
public:
  explicit ColorConnector(ros::NodeHandle &n);
  // returns a pair of vectors, first vector is blue line, second vector is
  // right line
  std::pair<std::vector<Node *>, std::vector<Node *>>
  get_color_lines(const std::vector<std::vector<double>> &cones,
                  const std_msgs::Header &header);
  Node *get_closest_node(const std::vector<std::vector<double>> &cones,
                         const std::vector<Node *> &line, double max_dist);

private:
  ros::NodeHandle n_;
};

} // namespace pathplanning
#endif