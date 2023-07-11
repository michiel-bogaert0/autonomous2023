
#include <sim_control/bicycle_model.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <math.h>
#include <tuple>

BicycleModel::BicycleModel()
{

  // nh = new ros::NodeHandle("~");
  ros::NodeHandle n("~");
  // ros::NodeHandle nh(n,"bicycle_model");

  this->a = 0;
  this->ohm = 0;
  this->v = 0;
  this->delta = 0;
  this->omega = 0;
  this->x = 0;
  this->y = 0;
  this->theta = 0;
  this->omega = 0;

  this->alpha = n.param<float>("model/alpha", 0.1);
  this->beta = n.param<float>("model/beta", 0.8);
  this->wheelbase = n.param<float>("model/wheelbase", 1.0);
};

void BicycleModel::reset()
{
  this->stop();

  this->x = 0;
  this->y = 0;
  this->theta = 0;
};

void BicycleModel::stop()
{
  this->a = 0;
  this->ohm = 0;
  this->v = 0;
  this->delta = 0;
  this->omega = 0;
};

std::tuple<float, float, float, float, float, float> BicycleModel::update(float dt, float driving_intention,
                                                                          float steering_intention)
{
  if (abs(driving_intention) < 0.001)
  {
    this->a = fabs(this->v) > 0.001 ? 0.0 - (this->v / fabs(this->v) * this->alpha) : 0.0;
  }
  else
  {
    this->a = driving_intention / fabs(driving_intention);
  }

  if (fabs(steering_intention) < 0.001)
  {
    this->ohm = fabs(this->delta) > 0.001 ? 0.0 - (this->delta / fabs(this->delta) * this->beta) : 0.0;
  }
  else
  {
    this->delta = 0.0;
  }

  // First calculate new speed and delta. Limits are applied in the HW interface
  this->delta += dt * this->ohm;
  this->v += dt * this->a;

  if (fabs(this->v) < 0.001)
  {
    this->v = 0.0;
  }
  if (fabs(this->delta) < 0.001)
  {
    this->delta = 0.0;
  }

  // Now position and theta (bicycle model)
  this->x += this->v * dt * cos(this->theta);
  this->y += this->v * dt * sin(this->theta);
  this->omega = this->v * tan(this->delta) / this->wheelbase;
  this->theta += dt * this->omega;

  return { this->x, this->y, this->theta, this->v, this->a, this->omega };
};