#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <Eigen/Core>

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.hpp"

namespace g2o {

// ---------------------------------------------------------------
// -------------------------- Pose Vertex-------------------------
// ---------------------------------------------------------------

class PoseVertex : public BaseVertex<3, SE2> {
public:
  PoseVertex();

  virtual void setToOriginImpl() { _estimate = SE2(); }

  virtual void oplusImpl(const double *update) {
    SE2 up(update[0], update[1], update[2]);
    _estimate *= up;
  }

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;
};

// ---------------------------------------------------------------
// ----------------------- Landmark Vertex------------------------
// ---------------------------------------------------------------

class LandmarkVertex : public BaseVertex<2, Eigen::Vector2d> {
public:
  LandmarkVertex();
  float beliefs[3];
  std::vector<uint32_t> merged_ids;

  void setColor(int c, float b);
  void addBeliefs(float b0, float b1, float b2);
  void addMergeId(int id);
  int getColor();

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
