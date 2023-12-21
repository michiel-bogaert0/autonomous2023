#ifndef UTILS_HPP
#define UTILS_HPP

#include <algorithm>
#include <array>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <cmath>
#include <numeric>
#include <vector>

namespace pathplanning {
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

class Point {
public:
  double x;
  double y;

  Point(double x_val, double y_val) : x(x_val), y(y_val) {}
};

// Define a utility function to calculate squared distance between 2 points
double distance_squared(double x1, double y1, double x2, double y2);

// Function to check if there is no collision between a node and a point with
// safety distance around cones
bool no_collision(const Node &parent, const std::vector<double> &point,
                  const std::vector<std::vector<double>> &cones,
                  double safety_dist_squared);

// Function to get the closest center points to an origin point
std::vector<std::vector<double>>
get_closest_center(const std::vector<std::vector<double>> &center_points,
                   size_t amount, const std::vector<double> &origin = {0, 0},
                   double max_distance = -1);

// Function to sort the center points based on their distance to the origin
// point
std::vector<std::vector<double>>
sort_closest_to(const std::vector<std::vector<double>> &center_points,
                const std::vector<double> &origin = {0, 0},
                double max_distance = -1);

double calculate_variance(const std::vector<double> &data);
double calculate_median(const std::vector<double> &data);

std::tuple<Point, Point, Point, Point>
extend_line_to_rectangle(Point point, double angle_radians, double length,
                         double width);

bool is_point_inside_rectangle(
    const std::tuple<Point, Point, Point, Point> &rectangle_points,
    Point point);
std::vector<bool> vectorized_is_point_inside_rectangle(
    const std::tuple<Point, Point, Point, Point> &rectangle_points,
    const std::vector<Point> &points);

bool check_if_feasible_child(
    const Node &parent, const std::vector<std::array<double, 2>> &path,
    std::vector<double> next_pos,
    const std::vector<std::vector<double>> &bad_points,
    const std::vector<std::vector<double>> &center_points,
    const std::vector<std::vector<double>> &cones, double max_angle_change,
    double safety_dist_squared, double rect_width, int bad_points_threshold,
    int center_points_threshold);

} // namespace pathplanning

#endif // UTILS_HPP