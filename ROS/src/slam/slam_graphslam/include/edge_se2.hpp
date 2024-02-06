#ifndef G2O_TUTORIAL_EDGE_SE2_H
#define G2O_TUTORIAL_EDGE_SE2_H

#include "g2o/core/base_binary_edge.h"
#include "vertex_se2.hpp"

namespace g2o {

class EdgeSE2 : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
public:
  EdgeSE2();

  void computeError() {
    const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
    const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);
    SE2 delta =
        _inverseMeasurement * (v1->estimate().inverse() * v2->estimate());
    _error = delta.toVector();
  }

  void setMeasurement(const SE2 &m) {
    _measurement = m;
    _inverseMeasurement = m.inverse();
  }

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

protected:
  SE2 _inverseMeasurement;
};

} // namespace g2o

#endif
