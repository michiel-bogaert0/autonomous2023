#include <utils.hpp>

namespace pathplanning {

double distance_squared(double x1, double y1, double x2, double y2) {
  return std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
}

bool no_collision(const Node &parent, const std::vector<double> &point,
                  const std::vector<std::vector<double>> &cones,
                  double safety_dist_squared) {
  double x1 = parent.x;
  double y1 = parent.y;
  double x2 = point[0];
  double y2 = point[1];

  for (const auto &cone : cones) {
    double xc = cone[0];
    double yc = cone[1];

    double t_numerator = (x1 - xc) * (x1 - x2) + (y1 - yc) * (y1 - y2);
    double t_denominator = std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);

    double t = t_numerator / t_denominator;
    t = std::max(0.0, std::min(1.0, t));

    double xp = x1 + (x2 - x1) * t;
    double yp = y1 + (y2 - y1) * t;

    double dist_squared = distance_squared(xc, yc, xp, yp);
    if (dist_squared < safety_dist_squared) {
      return false;
    }
  }

  return true;
}

std::vector<std::vector<double>>
get_closest_center(const std::vector<std::vector<double>> &center_points,
                   size_t amount, const std::vector<double> &origin,
                   double max_distance) {
  std::vector<double> distances_squared;
  distances_squared.reserve(center_points.size());

  for (const auto &center_point : center_points) {
    double dist_squared = distance_squared(origin[0], origin[1],
                                           center_point[0], center_point[1]);
    distances_squared.push_back(dist_squared);
  }

  std::vector<std::vector<double>> points_within_distance;
  for (size_t i = 0; i < center_points.size(); i++) {
    if (distances_squared[i] < max_distance * max_distance ||
        max_distance == -1) {
      points_within_distance.push_back(center_points[i]);
    }
  }

  // If there are not enough points
  if (points_within_distance.size() <= amount) {
    return points_within_distance;
  }

  // Get the 'amount' closest center points
  std::vector<size_t> ind(center_points.size());
  std::iota(ind.begin(), ind.end(), 0);

  std::partial_sort(ind.begin(), ind.begin() + amount, ind.end(),
                    [&distances_squared](size_t i1, size_t i2) {
                      return distances_squared[i1] < distances_squared[i2];
                    });

  std::vector<std::vector<double>> closest_center_points;
  closest_center_points.reserve(amount);
  for (size_t i = 0; i < amount; i++) {
    closest_center_points.push_back(center_points[ind[i]]);
  }

  return closest_center_points;
}

std::vector<std::vector<double>>
sort_closest_to(const std::vector<std::vector<double>> &center_points,
                const std::vector<double> &origin, double max_distance) {
  std::vector<double> distances_squared;
  distances_squared.reserve(center_points.size());

  for (const auto &center_point : center_points) {
    double dist_squared = distance_squared(origin[0], origin[1],
                                           center_point[0], center_point[1]);
    distances_squared.push_back(dist_squared);
  }

  std::vector<std::vector<double>> points_within_distance;
  std::vector<double> distances_squared_points_within_distance;
  for (size_t i = 0; i < center_points.size(); i++) {
    if (distances_squared[i] < max_distance * max_distance ||
        max_distance == -1) {
      points_within_distance.push_back(center_points[i]);
      distances_squared_points_within_distance.push_back(distances_squared[i]);
    }
  }

  std::vector<size_t> ind(points_within_distance.size());
  std::iota(ind.begin(), ind.end(), 0);

  std::sort(ind.begin(), ind.end(),
            [&distances_squared_points_within_distance](size_t i1, size_t i2) {
              return distances_squared_points_within_distance[i1] <
                     distances_squared_points_within_distance[i2];
            });

  std::vector<std::vector<double>> sorted_points_within_distance;
  sorted_points_within_distance.reserve(points_within_distance.size());
  for (size_t i = 0; i < points_within_distance.size(); i++) {
    sorted_points_within_distance.push_back(points_within_distance[ind[i]]);
  }

  return sorted_points_within_distance;
}

double calculate_variance(const std::vector<double> &data) {
  // Using Boost Accumulators to calculate variance
  namespace ba = boost::accumulators;
  ba::accumulator_set<double, ba::stats<ba::tag::variance>> acc;

  for (double value : data) {
    acc(value);
  }

  return ba::variance(acc);
}

double calculate_median(const std::vector<double> &data) {
  // Using Boost Accumulators to calculate median
  namespace ba = boost::accumulators;
  ba::accumulator_set<double, ba::stats<ba::tag::median>> acc;

  for (double value : data) {
    acc(value);
  }

  return ba::median(acc);
}

std::tuple<Point, Point, Point, Point>
extend_line_to_rectangle(Point point, double angle_radians, double length,
                         double width) {
  // Extract x and y coordinates from the 'point'
  double x = point.x;
  double y = point.y;

  // Calculate the endpoint of the line using the given angle and length
  double end_x = x + length * std::cos(angle_radians);
  double end_y = y + length * std::sin(angle_radians);

  // Calculate the perpendicular vectors to get the points of the rectangle
  // We'll assume a width of 1 (you can adjust this if you want a wider
  // rectangle)
  double dx = width * std::cos(angle_radians + M_PI / 2);
  double dy = width * std::sin(angle_radians + M_PI / 2);

  // Calculate the four points of the rectangle
  Point p1(x + dx, y + dy);
  Point p2(end_x + dx, end_y + dy);
  Point p3(end_x - dx, end_y - dy);
  Point p4(x - dx, y - dy);

  return std::make_tuple(p1, p2, p3, p4);
}

// Function to calculate the cross product of two points (vectors)
double cross_product(Point p1, Point p2, Point p3) {
  return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

// Function to check if a point is on the same side of two line segments
bool is_point_on_same_side(Point p1, Point p2, Point p3, Point p4,
                           Point point) {
  return (cross_product(p1, p2, point) * cross_product(p3, p4, point) >= 0 &&
          cross_product(p2, p3, point) * cross_product(p4, p1, point) >= 0);
}

// Function to check if a list of points is inside a rectangle
std::vector<bool> vectorized_is_point_inside_rectangle(
    const std::tuple<Point, Point, Point, Point> &rectangle_points,
    const std::vector<Point> &points) {
  Point p1 = std::get<0>(rectangle_points);
  Point p2 = std::get<1>(rectangle_points);
  Point p3 = std::get<2>(rectangle_points);
  Point p4 = std::get<3>(rectangle_points);

  std::vector<bool> result(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    result[i] = is_point_on_same_side(p1, p2, p3, p4, points[i]);
  }

  return result;
}

// Function to check if a point is inside a rectangle
bool is_point_inside_rectangle(
    const std::tuple<Point, Point, Point, Point> &rectangle_points,
    Point point) {
  Point p1 = std::get<0>(rectangle_points);
  Point p2 = std::get<1>(rectangle_points);
  Point p3 = std::get<2>(rectangle_points);
  Point p4 = std::get<3>(rectangle_points);

  return is_point_on_same_side(p1, p2, p3, p4, point) &&
         is_point_on_same_side(p2, p3, p4, p1, point);
}

// Function to check if a child node is feasible to be added to a path
bool check_if_feasible_child(
    const Node &parent, const std::vector<std::array<double, 2>> &path,
    std::vector<double> next_pos,
    const std::vector<std::vector<double>> &bad_points,
    const std::vector<std::vector<double>> &center_points,
    const std::vector<std::vector<double>> &cones, double max_angle_change,
    double safety_dist_squared, double rect_width, int bad_points_threshold,
    int center_points_threshold) {

  // Make sure we're not looping from the parent to itself
  if (std::abs(next_pos[0] - parent.x) < 1e-6 &&
      std::abs(next_pos[1] - parent.y) < 1e-6) {
    return false;
  }

  // Make sure we are not visiting nodes double
  if (std::any_of(path.begin(), path.end(),
                  [&](const std::array<double, 2> &point) {
                    return std::abs(point[0] - next_pos[0]) < 1e-6 &&
                           std::abs(point[1] - next_pos[1]) < 1e-6;
                  })) {
    return false;
  }

  double angle_node =
      std::atan2(next_pos[1] - parent.y, next_pos[0] - parent.x);
  double angle_change = angle_node - parent.angle;
  double distance_node =
      std::pow(parent.x - next_pos[0], 2) + std::pow(parent.y - next_pos[1], 2);

  double abs_angle_change =
      std::min(std::abs(angle_change), 2 * M_PI - std::abs(angle_change));

  // Check that the angle change is within bounds
  if (abs_angle_change > max_angle_change) {
    return false;
  }

  // Check we're not colliding with a cone
  if (!no_collision(parent, next_pos, cones, safety_dist_squared)) {
    return false;
  }

  // Don't allow coming close to bad points
  auto rectangle_points =
      extend_line_to_rectangle(Point(parent.x, parent.y), angle_node,
                               std::sqrt(distance_node), rect_width);
  int bad_points_crossings = 0;
  for (auto &bad_point : bad_points) {
    Point bad_point_as_point = {bad_point[0], bad_point[1]};
    if (is_point_inside_rectangle(rectangle_points, bad_point_as_point)) {
      bad_points_crossings++;
      if (bad_points_crossings > bad_points_threshold) {
        return false;
      }
    }
  }

  // Also don't allow skipping center points
  int center_points_crossings = 0;
  for (auto &center_point : center_points) {
    Point center_point_as_point = {center_point[0], center_point[1]};
    if (is_point_inside_rectangle(rectangle_points, center_point_as_point)) {
      center_points_crossings++;
      if (center_points_crossings > center_points_threshold) {
        return false;
      }
    }
  }

  return true;
}

} // namespace pathplanning
