
#include <sim_control/bicycle_model.h>
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
  phi = in_phi;
  alpha = in_alpha;

  // Angular velocity (of wheel) and steering angle
  zeta += phi * dt;       // Of STEERING JOINT

  double drag_acc = DC * pow(v, 2);
  double friction_acc = fabs(v) > 0.001 ? mu : 0.0;

  a = alpha * R - drag_acc - friction_acc;

  v += a * dt;  // v of CoG (+ drag + friction)
  ang_vel = v / R;

  ROS_DEBUG_STREAM("phi " << phi << " alpha " << alpha << " ang_vel " << ang_vel << " zeta " << zeta);
  ROS_DEBUG_STREAM("v " << v << " a " << a << " dt " << dt << " drag acc " << drag_acc);

  // Outputs (and intermediates)
  omega = v * tan(zeta) / L;
  theta += omega * dt;
  double x_vel = v * cos(theta);
  double y_vel = v * sin(theta);

  x += x_vel * dt;
  y += y_vel * dt;
};