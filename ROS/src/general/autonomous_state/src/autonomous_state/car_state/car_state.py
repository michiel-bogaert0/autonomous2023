from abc import ABC, abstractmethod
from enum import Enum

class carStateEnum(Enum):
    OFF = 0
    ON = 1
    AVAILABLE = 2
    UNAVAILABLE = 3
    ENGAGED = 4
    ARMED = 5
    ACTIVATED = 6,
    YELLOW_CONTINUOUS = 7,
    YELLOW_FLASH = 8,
    BLUE_CONTINUOUS = 9,
    BLUE_FLASH = 10,
    DONT_CARE = 11,

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
    def get_state():
        """
        Returns:
            object with the (physical) state of the car systems,
            like EBS and ASSI. See general docs for info about this state
        """
        pass

    @abstractmethod
    def set_state(new_state):
        pass