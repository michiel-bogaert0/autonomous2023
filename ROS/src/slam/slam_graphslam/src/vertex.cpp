#include "vertex.hpp"

using namespace Eigen;

namespace g2o {

// ---------------------------------------------------------------
// -------------------------- Pose Vertex-------------------------
// ---------------------------------------------------------------

PoseVertex::PoseVertex() : BaseVertex<3, SE2>() {}

bool PoseVertex::read(std::istream &is) {
  Eigen::Vector3d p;
  is >> p[0] >> p[1] >> p[2];
  _estimate.fromVector(p);
  return true;
}

bool PoseVertex::write(std::ostream &os) const {
  Eigen::Vector3d p = estimate().toVector();
  os << p[0] << " " << p[1] << " " << p[2];
  return os.good();
}

// ---------------------------------------------------------------
// ----------------------- Landmark Vertex------------------------
// ---------------------------------------------------------------

LandmarkVertex::LandmarkVertex() : BaseVertex<2, Vector2d>() {
  _estimate.setZero();
}

bool LandmarkVertex::read(std::istream &is) {
  is >> _estimate[0] >> _estimate[1];
  return true;
}

bool LandmarkVertex::write(std::ostream &os) const {
  os << estimate()(0) << " " << estimate()(1);
  return os.good();
}

} // namespace g2o
