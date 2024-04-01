#ifndef EDGE_HPP
#define EDGE_HPP

#include "g2o/core/base_binary_edge.h"
#include "vertex.hpp"

namespace g2o {

// ---------------------------------------------------------------
// ----------------------- Pose Edge -----------------------------
// ---------------------------------------------------------------

class PoseEdge : public BaseBinaryEdge<3, SE2, PoseVertex, PoseVertex> {
public:
  PoseEdge();

  void computeError() {
    const PoseVertex *v1 = static_cast<const PoseVertex *>(_vertices[0]);
    const PoseVertex *v2 = static_cast<const PoseVertex *>(_vertices[1]);
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

// ---------------------------------------------------------------
// ----------------------- Landmark Edge--------------------------
// ---------------------------------------------------------------

class LandmarkEdge
    : public BaseBinaryEdge<2, Eigen::Vector2d, PoseVertex, LandmarkVertex> {
public:
  LandmarkEdge();

  void computeError();

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;
};

} // namespace g2o

#endif
