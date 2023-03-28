from abc import ABC, abstractmethod

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from can_msgs.msg import Frame
from car_state import carStateEnum, CarState


class PegasusState(CarState):
    def __init__(self) -> None:

        rospy.Subscriber("/input/can", Frame, self.handle_can)

        self.state = {
            "TS": carStateEnum.OFF,
            "ASMS": carStateEnum.OFF,
            "R2D": carStateEnum.OFF,
            "SA": carStateEnum.OFF,
            "ASB": carStateEnum.OFF,
            "EBS": carStateEnum.OFF,
        }

    def handle_can(frame: Frame):
        """
        Handles incoming CAN message, but as a subscriber callback
        This way, we can put all HW/SW interfacing code in a single CAN driver.
        """

    def get_state():
        """
        Returns:
            object with the (physical) state of the car systems,
            like EBS and ASSI. See general docs for info about this state
        """
        pass
