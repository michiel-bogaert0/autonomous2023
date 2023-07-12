#ifndef BICYCLEMODEL_H
#define BICYCLEMODEL_H

#include <ros/ros.h>
#include <tuple>

class BicycleModel
{
public:
  BicycleModel();

  /**
   * Resets the internal state of the simulator
   */
  void reset();

  /**
   * Stop the car
   */
  void stop();

  // Joint states
  float get_steering_angle()
  {
    return this->delta;
  }
  float get_wheel_angular_velocity()
  {
    return this->v / this->R;
  }

  // Car state
  std::tuple<float, float, float> get_car_state()
  {
    return { x, y, theta };
  }
  float get_forward_velocity()
  {
    return this->v;
  }
  float get_angular_velocity()
  {
    return this->omega;
  }
  float get_acceleration() {
    return this->a;
  }

  /**
   * Should update internal state and motion model
   *
   * Args:
   *    dt: time since last update
   *    driving_intention: the intention of driving. +1 is "drive forward", -1 is "drive backwards". Can be anything inbetween
   *    steering_intention: the intention of steering. +1 is "steer left", -1 is "steer right". Can be anything inbetween
   *
   * Returns:
   *  (x, y, heading, forward vel, linear acceleration, angular acceleration)
   */
  std::tuple<float, float, float, float, float, float> update(float dt, float driving_intention,
                                                              float steering_intention);

  // Internal state
  /**
   * a: effort wheel (=acceleration)
   * delta: angle (=position) steering joint
   * 
   * v: velocity car
   * x, y: position car
   * theta: heading car
   * omega: angular velocity heading car
  */
  float a, ohm, v, delta, x, y, theta, omega;

  // Variables
  float alpha, beta, wheelbase, R;
};

#endif  // BICYCLEMODEL_H
