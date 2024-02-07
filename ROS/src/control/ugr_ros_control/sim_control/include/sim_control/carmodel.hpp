
#ifndef CARMODEL_H
#define CARMODEL_H

class CarModel
{
public:
  CarModel();

  /**
   * Resets the internal state of the simulator
   */
  virtual void reset();

  /**
   * Stop the car
   */
  virtual void stop();

  /**
   * Should update internal state and motion model
   *
   * Args:
   *    dt: time since last update
   *    driving_intention: the intention of driving. +1 is "drive forward", -1 is "drive backwards". Can be anything inbetween
   *    steering_intention: the intention of steering. +1 is "steer left", -1 is "steer right". Can be anything inbetween
   */
  virtual void update(float dt, float driving_intention, float steering_intention);
};

#endif  // CARMODEL_H
