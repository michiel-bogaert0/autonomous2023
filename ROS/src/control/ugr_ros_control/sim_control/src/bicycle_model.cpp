
#include <sim_control/bicycle_model.hpp>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <math.h>
#include <tuple>
#include <ros/ros.h>

BicycleModel::BicycleModel(double R, double L, double Lr, double mu, double DC) : R(R), L(L), Lr(Lr), mu(mu), DC(DC)
{
  this->reset();
};

void BicycleModel::reset()
{
  this->stop();

  this->x = 0;
  this->y = 0;
  this->theta = 0;
  this->zeta = 0;
  this->alpha = 0;
  this->ang_vel = 0;
  this->phi = 0;
  this->omega = 0;
};

void BicycleModel::stop()
{
  this->a = 0;
  this->phi = 0;
  this->v = 0;
};

void BicycleModel::update(double dt, double in_alpha, double in_phi)
{
  if (fabs(dt) < 0.0001)
  {
    return;
  }

  // Inputs
  alpha = in_alpha;
  phi = in_phi;

  // Angular velocity (of wheel) and steering angle

  double drag_acc = v > 0.1 ? DC * pow(v, 2) : 0.0;
  double friction_acc = v > 0.1 ? mu : 0.0;

  a = alpha * R - drag_acc - friction_acc;

  ang_vel = v / R;

  // Outputs (and intermediates)
  omega = v * tan(zeta) / L;

  double x_vel = v * cos(theta);
  double y_vel = v * sin(theta);

  x += x_vel * dt;
  y += y_vel * dt;
  v += a * dt;          // v of CoG (+ drag + friction)
  theta += omega * dt;  // Heading
  zeta += phi * dt;     // Of STEERING JOINT
};