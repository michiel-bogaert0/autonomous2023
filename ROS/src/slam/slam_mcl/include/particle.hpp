#ifndef __MCL_PARTICLE_H__
#define __MCL_PARTICLE_H__

#include <iostream>
#include <string>
#include <vector>

#include "kdtree.hpp"

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define LANDMARK_CLASS_COUNT 4

/**
 * Class representing a Particle in the MCL filter
 */
class Particle {
public:
  Particle();
  explicit Particle(double);
  Particle(float &w, VectorXf &pose, MatrixXf &variance);
  inline ~Particle(){};

  // getters
  float &w();
  MatrixXf &variance(); // pose variance
  VectorXf &pose();     // robot pose: x,y,theta (heading dir)
  float &prevyaw();     // Previous yaw
  int &rev();           // Revolutions

  // setters
  void setW(float w);
  void setPose(VectorXf &pose);
  void setPrevyaw(float yaw);
  void setRev(int rev);
  void incRev();
  void decRev();
  void setVariance(MatrixXf &variance);

private:
  float _w;
  VectorXf _pose;
  float _prev_yaw;
  int _rev;
  MatrixXf _variance;
};

#endif