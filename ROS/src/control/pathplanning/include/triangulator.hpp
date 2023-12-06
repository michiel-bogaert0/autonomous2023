#ifndef TRIANGULATOR_HPP
#define TRIANGULATOR_HPP

#include <center_points.hpp>
#include <paths.hpp>
#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <array>
#include <cmath>
#include <tuple>
#include <vector>

namespace pathplanning {
class Triangulator {

public:
  explicit Triangulator(ros::NodeHandle &n);

  std::pair<std::vector<Node *>, std::vector<std::vector<Node *>>>
  get_path(const std::vector<std::vector<double>> &cones,
           const std_msgs::Header &header);

private:
  ros::NodeHandle &n_;
  double triangulation_min_var_;
  double triangulation_var_threshold_;
  int max_iter_;
  double max_angle_change_;
  double max_path_distance_;
  double safety_dist_;
  double safety_dist_squared_;
  double stage1_rect_width_;
  int stage1_threshold_bad_points_;
  int stage1_threshold_center_points_;
  double stage2_rect_width_;
  int stage2_threshold_bad_points_;
  int stage2_threshold_center_points_;
  int max_depth_;
  double continuous_dist_;
  double range_front_;
  double range_behind_;
  double range_sides_;
  bool vis_;
  ros::Publisher vis_points_;
  ros::Publisher vis_lines_;
  std::string vis_namespace_;
  double vis_lifetime_;
  bool debug_visualisation_;

  TriangulationPaths triangulation_paths;

  void publish_points(const std::vector<std::vector<double>> &points,
                      const std_msgs::Header &header, const std::string &nsp,
                      const std::array<double, 4> &color, double scale);
  void publish_line(const std::vector<std::vector<double>> &line,
                    const std_msgs::Header &header);

  std::pair<std::vector<Node *>, std::vector<std::vector<Node *>>>
  get_best_path(const std::vector<Node *> &leaves,
                const std::vector<std::vector<double>> &cones);
};

} // namespace pathplanning

#endif