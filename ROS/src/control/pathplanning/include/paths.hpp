#include <algorithm>
#include <cmath>
#include <utils.hpp>
#include <vector>

namespace pathplanning {
class TriangulationPaths {
public:
  TriangulationPaths(int max_iter, double max_angle_change,
                     double max_path_distance, double safety_dist,
                     double stage1_rect_width_,
                     int stage1_threshold_bad_points_,
                     int stage1_threshold_center_points_,
                     double stage2_rect_width_,
                     int stage2_threshold_bad_points_,
                     int stage2_threshold_center_points_, int max_depth_,
                     double continuous_dist_, double close_path_dist)
      : max_iter(max_iter), max_angle_change(max_angle_change),
        max_path_distance(max_path_distance), safety_dist(safety_dist),
        safety_dist_squared(safety_dist * safety_dist),
        angle_sensitivity(M_PI / 9.0) {
    this->stage1_rect_width_ = stage1_rect_width_;
    this->stage1_threshold_bad_points_ = stage1_threshold_bad_points_;
    this->stage1_threshold_center_points_ = stage1_threshold_center_points_;
    this->stage2_rect_width_ = stage2_rect_width_;
    this->stage2_threshold_bad_points_ = stage2_threshold_bad_points_;
    this->stage2_threshold_center_points_ = stage2_threshold_center_points_;
    this->max_depth_ = max_depth_;
    this->continuous_dist_ = continuous_dist_;
    this->close_path_dist = close_path_dist;
  }

  std::pair<Node *, std::vector<Node *>>
  get_all_paths(const std::vector<std::vector<double>> &triangulation_centers,
                const std::vector<std::vector<double>> &center_points,
                const std::vector<std::vector<double>> &cones,
                double range_front);

  std::pair<double, double>
  get_cost_branch(const std::vector<Node *> &branch,
                  const std::vector<std::vector<double>> &cones,
                  double range_front);

private:
  int max_iter;
  double max_angle_change;
  double max_path_distance;
  double safety_dist;
  double safety_dist_squared;
  double angle_sensitivity;

  double stage1_rect_width_;
  int stage1_threshold_bad_points_;
  int stage1_threshold_center_points_;
  double stage2_rect_width_;
  int stage2_threshold_bad_points_;
  int stage2_threshold_center_points_;
  int max_depth_;
  double continuous_dist_;
  double close_path_dist;
};

} // namespace pathplanning