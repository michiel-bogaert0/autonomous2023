#!/usr/bin/env python3
import numpy as np
from .CarModel import CarModel
import rospy

class BicycleModel(CarModel):
    
    def __init__(self) -> None:
        
        """
        Bicycle model

        Inputs:
          a: linear acceleration
          Ohm: angular turning speed of steering wheel

        Outputs (state):
          x: x-position of back wheel 
          y: y-position of back wheel
          theta: heading to x-as

        Args:
          alpha: linear velocity 'friction' factor
          beta: steering 'homing' factor
          L: wheelbase length
          input_scale: input scaling 

        """
        
        super().__init__()

        # ROS params
        self.L = rospy.get_param("~wheelbase", 1.0)
        self.delta_max = rospy.get_param("~max_steering_angle", 0.8)
        self.v_max = rospy.get_param("~max_speed", 5.0)
        self.alpha = rospy.get_param("~alpha", 0.1)
        self.beta = rospy.get_param("~beta", 0.8)
        self.input_scale = rospy.get_param("~input_scale", [1.0, 1.0])

        # Internal state
        self.a = 0
        self.ohm = 0
        self.v = 0
        self.delta = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.omega = 0

    def stop(self):
        """
        Stops the car
        """
        self.a = 0
        self.ohm = 0
        self.v = 0
        self.delta = 0
        self.omega = 0
        
    def reset(self):
        """
        Resets the car to initial (0, 0) position
        """
        self.stop()

        self.x = 0
        self.y = 0
        self.theta = 0
    
    def update(self, dt, driving_intention, steering_intention):
        """
        Should update internal state and motion model

        Args:
            dt: time since last update
            driving_intention: the intention of driving. +1 is "drive forward", -1 is "drive backwards". Can be anything inbetween
            steering_intention: the intention of steering. +1 is "steer left", -1 is "steer right". Can be anything inbetween
        
            
        Returns: 
          Note: should be in our frame conventions
  
          (x, y, heading), (forward velocity, linear acceleration, angular acceleration)
        """

        if abs(driving_intention) < 0.001:
            self.a = 0.0 - (self.v / abs(self.v) * self.alpha if abs(self.v) > 0.001 else 0.0)
        else:
            self.a = driving_intention / abs(driving_intention)

        if abs(steering_intention) < 0.001:
            self.ohm = 0.0 - ( self.delta / abs(self.delta) * self.beta if abs(self.delta) > 0.001 else 0.0)
        else:
            self.ohm = steering_intention / abs(steering_intention)

        # First calculate new speed and delta
        self.delta += self.input_scale[1] * dt * self.ohm
        self.delta = max(-self.delta_max, min(self.delta, self.delta_max))

        self.v += self.input_scale[0] * dt * self.a
        self.v = max(-self.v_max, min(self.v, self.v_max))

        if abs(self.v) < 0.001:
            self.v = 0.0
        if abs(self.delta) < 0.001:
            self.delta = 0.0

        # Now position and theta (bicycle model)
        self.x += self.v * dt * np.cos(self.theta)
        self.y += self.v * dt * np.sin(self.theta)
        self.omega = self.v * np.tan(self.delta) / self.L
        self.theta += dt * self.omega

        return (self.x, self.y, self.theta), (self.v, self.a, self.ohm)