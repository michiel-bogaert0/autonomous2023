
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <iterator>

#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "particle.hpp"

#include <ros/ros.h>

using namespace std;
using namespace Eigen;

Particle::Particle()
{
  _pose = VectorXf(3);
  _variance = MatrixXf(3, 3);
  _pose.setZero(3);
  _variance.setZero(3, 3);

  _w = 1.0;
  _prev_yaw = 0.0;
  _rev = 0;
}

Particle::Particle(float &w, VectorXf &pose, MatrixXf &variance)
{
  _w = w;
  _pose = pose;
  _variance = variance;

  _prev_yaw = 0.0;
  _rev = 0;
}

// getters
float &Particle::w()
{
  return _w;
}

VectorXf &Particle::pose()
{
  return _pose;
}

float &Particle::prevyaw()
{
  return _prev_yaw;
}

int &Particle::rev()
{
  return _rev;
}

MatrixXf &Particle::variance()
{
  return _variance;
}

// setters
void Particle::setW(float w)
{
  _w = w;
}
void Particle::setPose(VectorXf &pose)
{
  _pose = pose;
}
void Particle::setPrevyaw(float yaw)
{
  _prev_yaw = yaw;
}
void Particle::setRev(int rev)
{
  _rev = rev;
}
void Particle::incRev()
{
  _rev++;
}
void Particle::decRev()
{
  _rev--;
}
void Particle::setVariance(MatrixXf &variance)
{
  _variance = variance;
}