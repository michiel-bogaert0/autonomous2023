#! /usr/bin/python3

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from node_fixture import create_diagnostic_message
from node_fixture.node_manager import ManagedNode
from orion_manual_state import OrionManualState


class ManualController(ManagedNode):  # maybe NodeManager
    def __init__(self) -> None:
        super().__init__("manual_state")
        self.spin()

    def doActivate(self):
        self.ccs = {}

        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.car_name = rospy.get_param("car", "orion")

        if self.car_name == "orion":
            self.car = OrionManualState()
        else:
            raise f"Unknown model! (model given to manual controller was: '{self.car_name}')"

    def active(self):
        # Gets car state as reported by our helper class
        self.ccs = self.car.get_state()

        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.OK, "[GNRL] STATE: Car state", str(self.ccs)
            )
        )


node = ManualController()
