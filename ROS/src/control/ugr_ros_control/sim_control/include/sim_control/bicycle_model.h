#ifndef BICYCLEMODEL_H
#define BICYCLEMODEL_H

#include <ros/ros.h>
#include <tuple>

class BicycleModel
{
public:
  /**
   * Based on
   * "https://thef1clan.com/2020/09/21/vehicle-dynamics-the-kinematic-bicycle-model/#:~:text=What%20is%20the%20kinematic%20bicycle,wheels%20and%201%20steering%20angle."
   * But with added simple model for acceleration (drive effort) -> velocity instead of velocity as input
   */
  BicycleModel(double R, double L, double Lr, double mu, double DC);

  /**
   * Resets the internal state of the simulator
   */
  void reset();

  /**
   * Stop the car
   */
  void stop();

  // Joint states
  double get_steering_angle()
  {
    return this->zeta;
  }
  double get_wheel_angular_velocity()
  {
    return this->ang_vel;
  }

  // Car state
  std::tuple<double, double, double> get_car_state()
  {
    return { x, y, theta };
  }
  double get_forward_velocity()
  {
    return this->v;
  }
  double get_angular_velocity()
  {
    return this->omega;
  }
  double get_acceleration()
  {
    return this->a;
  }

  /**
   * Should update internal state and motion model
   *
   * Args:
   *    dt: time since last update
   *    driving_intention: the intention of driving. +1 is "drive forward", -1 is "drive backwards". Can be anything inbetween
   *    steering_intention: the intention of steering. +1 is "steer left", -1 is "steer right". Can be anything inbetween
   */
  void update(double dt, double in_a, double in_phi);

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

  /**
   * Inputs
   *
   * a: angular acceleration
   * phi: steering rate of change
   */
  double alpha, phi = 0;

  /**
   * Outputs
   *
   * omega: change of CoG heading
   * v: CoG velocity
   * x, y: CoG position
   * theta: CoG heading
   * zeta: steering angle
   * ang_vel: angular velocity wheels
   * a: linear acceleration
   */
  double v, x, y, theta, zeta, omega, ang_vel, a = 0;

  // Variables
  double R, L, Lr, DC, mu;
};

#endif  // BICYCLEMODEL_H
