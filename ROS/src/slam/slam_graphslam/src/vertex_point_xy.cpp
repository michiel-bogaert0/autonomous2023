#include "vertex_point_xy.hpp"

using namespace Eigen;

namespace g2o {

VertexPointXY::VertexPointXY() : BaseVertex<2, Vector2d>() {
  _estimate.setZero();
}

bool VertexPointXY::read(std::istream &is) {
  is >> _estimate[0] >> _estimate[1];
  return true;
}

bool VertexPointXY::write(std::ostream &os) const {
  os << estimate()(0) << " " << estimate()(1);
  return os.good();
}

} // namespace g2o
