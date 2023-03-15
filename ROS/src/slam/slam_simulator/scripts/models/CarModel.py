#!/usr/bin/env python3
from abc import ABC, abstractmethod
import numpy as np

class CarModel(ABC):
    
    def __init__(self) -> None:

        # Be sure to add all ROS parameters here!

        self.reset()

    @abstractmethod
    def reset(self):
        pass
        
    @abstractmethod
    def stop(self):
        pass
 
    @absractmethod
    def update(self, dt, driving_intention, steering_intention):
        """
        Should update internal state and motion model

        Args:
            dt: time since last update
            driving_intention: the intention of driving. +1 is "drive forward", -1 is "drive backwards". Can be anything inbetween
            steering_intention: the intention of steering. +1 is "steer left", -1 is "steer right". Can be anything inbetween
        
            
        Returns: 
          Car state (according to our frame conventions): x, y, heading (yaw)
        """
        
        pass
        
        # First calculate new speed and delta
        self.delta += self.input_scale[1] * dt * steering_intention
        self.delta = max(-self.delta_max, min(self.delta, self.delta_max))

        self.v += self.input_scale[0] * dt * driving_intention
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