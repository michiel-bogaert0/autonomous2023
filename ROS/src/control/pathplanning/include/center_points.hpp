#ifndef TRIANGULATION_HPP
#define TRIANGULATION_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <delaunay.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <ros/ros.h>
#include <unordered_set>
#include <utils.hpp>
#include <vector>

namespace pathplanning {

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>,
           std::vector<std::vector<double>>>
get_center_points(const std::vector<std::vector<double>> &position_cones,
                  const std::vector<int> &classes, double triangulation_max_var,
                  double triangulation_var_threshold, double range_front,
                  bool color);

} // namespace pathplanning

#endif // TRIANGULATION_HPP