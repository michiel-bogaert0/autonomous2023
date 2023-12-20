#ifndef UTILS_HPP
#define UTILS_HPP

#include <algorithm>
#include <array>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <cmath>
#include <numeric>
#include <vector>

namespace boundaryestimation {

class Node {
public:
  double x;
  double y;
  double distance;
  Node *parent;                 // Pointer to the parent node
  std::vector<Node *> children; // Vector of pointers to child nodes
  double angle;
  double angle_change;

  Node(double x, double y, double distance, Node *parent,
       const std::vector<Node *> &children, double angle, double angle_change)
      : x(x), y(y), distance(distance), parent(parent), children(children),
        angle(angle), angle_change(angle_change) {}
};

// Define a utility function to calculate squared distance between 2 points
double distance_squared(double x1, double y1, double x2, double y2);

// Define a utility function to calculate distance between 2 points
double angle_between(double x1, double y1, double x2, double y2);

// Define a utility function to calculate difference between 2 angles
double angle_difference(double angle1, double angle2);

} // namespace boundaryestimation

#endif // UTILS_HPP