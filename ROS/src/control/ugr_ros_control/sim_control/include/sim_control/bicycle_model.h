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

  float get_steering_angle() {
    return this->delta;
  }

  float get_forward_velocity() {
    return this->get_forward_velocity();
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
  std::tuple<float, float, float, float, float, float> update(float dt, float driving_intention, float steering_intention);

private:

  // Internal state
  float a, ohm, v, delta, x, y, theta, omega;

  // Variables
  float alpha, beta, wheelbase;
};

#endif  // BICYCLEMODEL_H
