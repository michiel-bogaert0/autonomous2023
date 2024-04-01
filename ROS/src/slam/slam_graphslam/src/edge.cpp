#include "edge.hpp"

using namespace Eigen;

namespace g2o {

// ---------------------------------------------------------------
// ----------------------- Pose Edge -----------------------------
// ---------------------------------------------------------------

PoseEdge::PoseEdge() : BaseBinaryEdge<3, SE2, PoseVertex, PoseVertex>() {}

bool PoseEdge::read(std::istream &is) {
  Vector3d p;
  is >> p[0] >> p[1] >> p[2];
  _measurement.fromVector(p);
  _inverseMeasurement = measurement().inverse();
  for (int i = 0; i < 3; ++i)
    for (int j = i; j < 3; ++j) {
      is >> information()(i, j);
      if (i != j)
        information()(j, i) = information()(i, j);
    }
  return true;
}

bool PoseEdge::write(std::ostream &os) const {
  Vector3d p = measurement().toVector();
  os << p.x() << " " << p.y() << " " << p.z();
  for (int i = 0; i < 3; ++i)
    for (int j = i; j < 3; ++j)
      os << " " << information()(i, j);
  return os.good();
}

// ---------------------------------------------------------------
// ----------------------- Landmark Edge--------------------------
// ---------------------------------------------------------------

LandmarkEdge::LandmarkEdge()
    : BaseBinaryEdge<2, Vector2d, PoseVertex, LandmarkVertex>() {}

bool LandmarkEdge::read(std::istream &is) {
  is >> _measurement[0] >> _measurement[1];
  is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
  information()(1, 0) = information()(0, 1);
  return true;
}

bool LandmarkEdge::write(std::ostream &os) const {
  os << measurement()[0] << " " << measurement()[1] << " ";
  os << information()(0, 0) << " " << information()(0, 1) << " "
     << information()(1, 1);
  return os.good();
}

void LandmarkEdge::computeError() {
  const PoseVertex *v1 = static_cast<const PoseVertex *>(_vertices[0]);
  const LandmarkVertex *l2 = static_cast<const LandmarkVertex *>(_vertices[1]);
  _error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
}

} // namespace g2o
