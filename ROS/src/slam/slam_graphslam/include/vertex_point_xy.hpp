#ifndef G2O_TUTORIAL_VERTEX_POINT_XY_H
#define G2O_TUTORIAL_VERTEX_POINT_XY_H

#include <Eigen/Core>

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

namespace g2o {

class VertexPointXY : public BaseVertex<2, Eigen::Vector2d> {
public:
  VertexPointXY();

  virtual void setToOriginImpl() { _estimate.setZero(); }

  virtual void oplusImpl(const double *update) {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
  }

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;
};

} // namespace g2o

#endif
