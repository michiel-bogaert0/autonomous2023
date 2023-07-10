#! /usr/bin/python3
import rospy
from enum import Enum
from ugr_msgs.msg import State
from node_launcher import NodeLauncher
from node_fixture.node_fixture import (
    AutonomousMission,
    SLAMStatesEnum,
    AutonomousStatesEnum,
    StateMachineScopeEnum,
    create_diagnostic_message,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from time import sleep

class Controller:
    def __init__(self) -> None:
        """
        Control controller
        """
        rospy.init_node("control_controller")

        self.state = SLAMStatesEnum.IDLE

        self.launcher = NodeLauncher()
        self.mission = ""

        rospy.Subscriber("/state", State, self.handle_state_change)

        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        while not rospy.is_shutdown():
            
            try:
                self.launcher.run()
                self.diagnostics_publisher.publish(
                    create_diagnostic_message(
                        DiagnosticStatus.OK, "[CTRL] Node launching", ""
                    )
                )
            except Exception as e:
                self.diagnostics_publisher.publish(
                    create_diagnostic_message(
                        DiagnosticStatus.ERROR, "[CTRL] Node launching", str(e)
                    )
                )
            self.update()

            sleep(0.1)

    def update(self):
        """
        Updates the internal state and launches or kills nodes if needed
        """

        new_state = self.state

        if self.state == SLAMStatesEnum.IDLE or self.state == SLAMStatesEnum.FINISHED or (rospy.has_param("/mission") and rospy.get_param("/mission") != self.mission):
            if rospy.has_param("/mission") and rospy.get_param("/mission") != "":
                # Go to state depending on mission
                self.mission = rospy.get_param("/mission")

                if self.mission == AutonomousMission.ACCELERATION:
                    new_state = SLAMStatesEnum.RACING
                elif self.mission == AutonomousMission.SKIDPAD:
                    new_state = SLAMStatesEnum.RACING
                elif self.mission == AutonomousMission.AUTOCROSS:
                    new_state = SLAMStatesEnum.EXPLORATION
                elif self.mission == AutonomousMission.TRACKDRIVE:
                    new_state = SLAMStatesEnum.EXPLORATION
                else:
                    new_state = SLAMStatesEnum.EXPLORATION

                self.launcher.launch_node(
                    "control_controller", f"launch/{self.mission}_{new_state}.launch"
                )
            else:
                self.launcher.shutdown()
        elif not rospy.has_param("/mission"):
            self.launcher.shutdown()
            new_state = SLAMStatesEnum.IDLE

        self.change_state(new_state)

    def handle_state_change(self, state: State):
        """
        Handles state transition from other state machines

        Args:
            state: the state transition
        """

        if state.scope == StateMachineScopeEnum.SLAM:
            self.state = state.cure_state
            
            if self.state == SLAMStatesEnum.FINISHING:
                #TODO: put velocity to 0 and steering to ??
                print("finishing")
            elif self.state == SLAMStatesEnum.EXPLORATION or self.state == SLAMStatesEnum.RACING:
                self.launcher.launch_node(
                    "control_controller", f"launch/{self.mission}_{self.state}.launch"
                )
            else:
                self.launcher.shutdown()

node = Controller()