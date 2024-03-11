
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
  ROS_INFO_STREAM_THROTTLE(0.05, "BicycleModel::update: dt: " << dt << " in_alpha: " << in_alpha << " in_phi: " << in_phi);

  // Inputs
  alpha = in_alpha;
  phi = in_phi;

  // Angular velocity (of wheel) and steering angle

  double drag_acc = v > 0.1 ? DC * pow(v, 2) : 0.0;
  double friction_acc = v > 0.1 ? mu : 0.0;

  // drag_acc = 0.0;
  // friction_acc = 0.0;

  a = alpha * R; //- drag_acc - friction_acc;

  

  // Outputs (and intermediates)
  omega = v * tan(zeta) / L;

  double x_vel = v * cos(theta);
  double y_vel = v * sin(theta);

  x += x_vel * dt;
  y += y_vel * dt;
  v += a * dt;          // v of CoG (+ drag + friction)
  ang_vel = v / R;
  theta += omega * dt;  // Heading
  zeta += phi * dt;     // Of STEERING JOINT
  // ROS_INFO_STREAM("BicycleModel::update: delta_x: " << x_vel * dt << " delta_y: " << y_vel * dt << " delta_v: " << a * dt << " delta_theta: " << omega * dt << " delta_zeta: " << phi * dt);
  // ROS_INFO_STREAM("zeta: " << zeta);
  // zeta = std::min(std::max(zeta, -M_PI / 4), M_PI / 4);
};