
#include <sim_control/bicycle_model.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <math.h>
#include <tuple>
#include <ros/ros.h>

BicycleModel::BicycleModel(double R, double L, double Lr, double mu) : R(R), L(L), Lr(Lr), mu(mu)
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

  if (fabs(dt) < 0.0001) {
    return;
  }

  // Inputs
  phi = in_phi;
  alpha = in_alpha;

  // Angular velocity (of wheel) and steering angle
  ang_vel += alpha * dt;  // Of WHEEL
  zeta += phi * dt;       // Of STEERING JOINT

  ROS_INFO_STREAM("phi " << phi << " alpha " << alpha << " ang_vel " << ang_vel << " zeta " << zeta);

  double drag_acc = mu * pow(v, 2);

  a = alpha * R - drag_acc;

  v += a * dt;  // v of CoG (+ drag)

  ROS_INFO_STREAM("v " << v << " a " << a << " dt " << dt << " drag acc " << drag_acc);

  // Outputs (and intermediates)
  double beta = atan(tan(zeta) / L * Lr);

  omega = v * tan(zeta) * cos(beta) / L;
  theta += omega * dt;
  double x_vel = v * cos(theta + beta);
  double y_vel = v * sin(theta + beta);

  x += x_vel * dt;
  y += y_vel * dt;
};