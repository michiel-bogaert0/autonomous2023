#ifndef G2O_TUTORIAL_EDGE_SE2_POINT_XY_H
#define G2O_TUTORIAL_EDGE_SE2_POINT_XY_H

#include "g2o/core/base_binary_edge.h"
#include "vertex_point_xy.hpp"
#include "vertex_se2.hpp"

namespace g2o {

class EdgeSE2PointXY
    : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXY> {
public:
  EdgeSE2PointXY();

  void computeError();

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;
};
} // namespace g2o

#endif
