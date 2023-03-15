#!/usr/bin/env python3
from abc import ABC, abstractmethod

class CarModel(ABC):
    
    def __init__(self) -> None:
        """
        Car model abstract class

        Be sure to add ROS parameters and internal state.
        Except for abstract methods, you are free to implement whatever.
        No guarantees can be made about speed of calling update. Max is 100FPS
        
        In order to make a model, extend this class in a new file. 
        Don't forget to update __init__.py and the main car.py file
        """

        self.reset()

    @abstractmethod
    def reset(self):
        """
        Resets the car to initial (0, 0) position
        """
        pass
        
    @abstractmethod
    def stop(self):
        """
        Stops the car
        """
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