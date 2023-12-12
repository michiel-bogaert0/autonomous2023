#ifndef __KDTREEPOINT_H__
#define __KDTREEPOINT_H__

#include <Eigen/Core>

/**
 * KDtree point class
 */

class KDTreePoint {
public:
  KDTreePoint(Eigen::VectorXf &mean, int id) : mean(mean), id(id){};

  double operator[](const int index) const {
    if (index == 0)
      return this->mean(0);
    else if (index == 1)
      return this->mean(1);
    else
      throw std::invalid_argument("Index must be 0 or 1");
  };

  int getId() { return this->id; };

  Eigen::VectorXf &getMean() { return this->mean; };

  static const int DIM = 2;

private:
  Eigen::VectorXf mean;
  int id;
};

#endif