#ifndef TRIANGULATION_HPP
#define TRIANGULATION_HPP

#include <vector>
#include <array>
#include <cmath>
#include <queue>
#include <algorithm>
#include <iostream>
#include <unordered_set>
#include <utils.hpp>
#include <delaunay.hpp>

namespace triangulation {

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>>
get_center_points(const std::vector<std::vector<double>>& position_cones,
                  double triangulation_min_var,
                  double triangulation_var_threshold,
                  double range_front);

std::vector<std::vector<double>> filter_center_points(const std::vector<std::vector<double>>& center_points,
                                                       const std::vector<std::vector<double>>& triangulation_centers,
                                                       const std::vector<std::vector<double>>& cones);

} // namespace triangulation

#endif // TRIANGULATION_HPP