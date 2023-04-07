from abc import ABC, abstractmethod
from enum import Enum

"""
Explanation of system states (hierarchical):

- OFF: the system is verified to be off
- ON: the system is verified to be on
- ACTIVATED: the system is verified to be explicitly activated

- UNKNOWN: status unkown
"""

class carStateEnum(Enum):
    OFF = 0
    ON = 1
    ACTIVATED = 2,
    UNKOWN = 3,

class CarState(ABC):
    
    """
    Autonomous systems:

    - TS: Tractive System status
    - ASMS: Autonomous System Master Switch
    - R2D: Ready 2 Drive
    - SA: Steering Actuator
    - ASB: Autonomous System Brake
    - EBS: Emergency Brake System
    - GO: "GO" signal

    See Enum above for status stuff

    """

    def __init__(self) -> None:
        pass
    
    @abstractmethod
    def get_state(self):
        """
        Returns:
            object with the (physical) state of the car systems,
            like EBS and ASSI. See general docs for info about this state
        """
        pass

    @abstractmethod
    def update(self, state):
        pass