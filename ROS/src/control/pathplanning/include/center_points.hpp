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

std::tuple<std::vector<std::array<double, 2>>, std::vector<std::array<double, 2>>>
get_center_points(const std::vector<std::array<double, 2>>& position_cones,
                  double triangulation_min_var,
                  double triangulation_var_threshold,
                  double range_front);

std::vector<std::array<double, 2>> filter_center_points(const std::vector<std::array<double, 2>>& center_points,
                                                       const std::vector<std::array<double, 2>>& triangulation_centers,
                                                       const std::vector<std::array<double, 3>>& cones);

} // namespace triangulation

#endif // TRIANGULATION_HPP