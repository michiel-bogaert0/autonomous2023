#include <utils.hpp>

namespace boundaryestimation {

double distance_squared(double x1, double y1, double x2, double y2) {
  return std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
}

double angle_between(double x1, double y1, double x2, double y2) {
  return std::atan2(y2 - y1, x2 - x1);
}

double angle_difference(double angle1, double angle2) {
  double angle = std::abs(angle1 - angle2);
  angle = std::min(angle, 2 * M_PI - angle);
  return angle;
}

} // namespace boundaryestimation
