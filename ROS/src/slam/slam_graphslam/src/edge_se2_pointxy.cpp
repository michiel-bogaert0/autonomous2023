#include "edge_se2_pointxy.hpp"

using namespace Eigen;

namespace g2o {

EdgeSE2PointXY::EdgeSE2PointXY()
    : BaseBinaryEdge<2, Vector2d, VertexSE2, VertexPointXY>() {}

bool EdgeSE2PointXY::read(std::istream &is) {
  is >> _measurement[0] >> _measurement[1];
  is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
  information()(1, 0) = information()(0, 1);
  return true;
}

bool EdgeSE2PointXY::write(std::ostream &os) const {
  os << measurement()[0] << " " << measurement()[1] << " ";
  os << information()(0, 0) << " " << information()(0, 1) << " "
     << information()(1, 1);
  return os.good();
}

void EdgeSE2PointXY::computeError() {
  const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
  const VertexPointXY *l2 = static_cast<const VertexPointXY *>(_vertices[1]);
  _error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
}

} // namespace g2o
