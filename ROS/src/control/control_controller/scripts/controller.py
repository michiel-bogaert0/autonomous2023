#! /usr/bin/python3
import rospy
from enum import Enum
from ugr_msgs.msg import State
from node_launcher.node_launcher import NodeLauncher
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
        self.car = ""

        self.car = rospy.get_param("/car")

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

            sleep(1)

    def handle_state_change(self, state: State):
        """
        Handles state transition from other state machines

        Args:
            state: the state transition
        """

        if rospy.has_param("/mission") and rospy.get_param("/mission") != "":
            # Go to state depending on mission
            self.mission = rospy.get_param("/mission")
        else:
            self.launcher.shutdown()
            return

        if state.scope == StateMachineScopeEnum.SLAM:
            self.state = state.cur_state

            if self.state == SLAMStatesEnum.FINISHED:
                rospy.set_param("/pure_pursuit/speed/target", 0.0)

            elif self.state == SLAMStatesEnum.EXPLORATION or self.state == SLAMStatesEnum.RACING:
                self.launcher.launch_node(
                    "control_controller", f"launch/{self.mission}_{self.state}.launch"
                )
                
                speed_target = 0.0
                if self.state == SLAMStatesEnum.EXPLORATION:
                    speed_target = 3.0 if self.car == 'simulation' else 1.0

                elif self.state == SLAMStatesEnum.RACING:
                    if self.mission == 'trackdrive' or self.mission == 'skidpad':
                        speed_target = 4.0 if self.car == 'simulation' else 0.8

                    elif self.mission == 'acceleration':
                        speed_target = 4.0 if self.car == 'simulation' else 3.0

                rospy.set_param("/pure_pursuit/speed/target", speed_target)

            else:
                self.launcher.shutdown()

node = Controller()