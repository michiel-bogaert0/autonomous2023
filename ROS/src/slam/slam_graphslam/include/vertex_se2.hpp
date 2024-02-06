#ifndef G2O_TUTORIAL_VERTEX_SE2_H
#define G2O_TUTORIAL_VERTEX_SE2_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.hpp"

namespace g2o {

class VertexSE2 : public BaseVertex<3, SE2> {
public:
  VertexSE2();

  virtual void setToOriginImpl() { _estimate = SE2(); }

  virtual void oplusImpl(const double *update) {
    SE2 up(update[0], update[1], update[2]);
    _estimate *= up;
  }

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;
};

} // namespace g2o

#endif
