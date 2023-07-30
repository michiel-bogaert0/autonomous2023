#include <vector>
#include <algorithm>
#include <cmath>
#include <utils.hpp>

namespace triangulation {
class TriangulationPaths {
public:
    TriangulationPaths(int max_iter, double max_angle_change, double max_path_distance, double safety_dist)
        : max_iter(max_iter), max_angle_change(max_angle_change), max_path_distance(max_path_distance), safety_dist(safety_dist), safety_dist_squared(safety_dist * safety_dist), angle_sensitivity(M_PI / 9.0) {}

    std::pair<Node*, std::vector<Node*>> get_all_paths(
        const std::vector<std::vector<double>>& triangulation_centers,
        const std::vector<std::vector<double>>& center_points,
        const std::vector<std::vector<double>>& cones,
        double range_front
    );

    std::pair<double, double> get_cost_branch(const std::vector<Node*>& branch, const std::vector<std::vector<double>>& cones, double range_front);

private:
    int max_iter;
    double max_angle_change;
    double max_path_distance;
    double safety_dist;
    double safety_dist_squared;
    double angle_sensitivity;
};

} // namespace triangulation