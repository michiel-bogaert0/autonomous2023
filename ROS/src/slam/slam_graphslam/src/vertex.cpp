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
  this->beliefs[0] = 0.0f;
  this->beliefs[1] = 0.0f;
  this->beliefs[2] = 0.0f;
}

void LandmarkVertex::setColor(int c, float b) {
  for (int i = 0; i < 3; i++) {
    // int new_belief = 1 - b;
    int new_belief = 0.0f;
    if (i == c) {
      new_belief = b;
    }
    if (fabs(this->beliefs[i] + new_belief) < 0.01f) {
      this->beliefs[i] = 0.0f;
    } else {
      this->beliefs[i] = (this->beliefs[i] + new_belief) / 2.0f;
    }
  }
}

int LandmarkVertex::getColor() {
  float max_belief = this->beliefs[0];
  int max_index = 0;
  for (int i = 1; i < 3; i++) {
    if (this->beliefs[i] > max_belief) {
      max_belief = this->beliefs[i];
      max_index = i;
    }
  }
  return max_index;
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
