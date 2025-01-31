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

struct PointInfo {
  std::vector<double> point;
  double angle;
  double angle_change;
  double distance_squared;
};

class TrianglePoint {
public:
  double x;
  double y;
  uint8_t colorIndex;

  TrianglePoint(double x_val, double y_val, uint8_t colorindex)
      : x(x_val), y(y_val), colorIndex(colorindex) {}
};

class Triangle {
public:
  std::array<TrianglePoint, 3> points;
  std::array<double, 3> sides;

  Triangle(const TrianglePoint &point1, const TrianglePoint &point2,
           const TrianglePoint &point3)
      : points{point1, point2, point3} {
    sides[0] = distance(points[0], points[1]);
    sides[1] = distance(points[1], points[2]);
    sides[2] = distance(points[2], points[0]);
  }

private:
  double distance(const TrianglePoint &pt1, const TrianglePoint &pt2) const {
    return std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2);
  }
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

// Function to sort the center points based on their angle change from the
// origin point
std::vector<PointInfo>
sort_by_angle_change(const std::vector<std::vector<double>> &center_points,
                     const std::vector<double> &origin, double original_angle,
                     double max_angle_change, double max_distance = -1);

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