#! /usr/bin/python3
from time import sleep

import rospy
from node_fixture.fixture import (
    AutonomousMission,
    DiagnosticArray,
    DiagnosticStatus,
    SLAMStatesEnum,
    StateMachineScopeEnum,
    create_diagnostic_message,
)
from node_fixture.node_management import (
    configure_node,
    set_state_active,
    set_state_finalized,
)
from node_launcher.node_launcher import NodeLauncher
from ugr_msgs.msg import State


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

        self.switch_time = rospy.Time.now().to_sec()

        rospy.Subscriber("/state", State, self.handle_state_change)

        # Diagnostics Publisher
        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        while not rospy.is_shutdown():
            try:
                self.launcher.run()
                self.diagnostics_pub.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.OK,
                        name="[CTRL CTRL] Node launching",
                        message="",
                    )
                )
            except Exception as e:
                self.diagnostics_pub.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.ERROR,
                        name="[CTRL CTRL] Node launching",
                        message=str(e),
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
            # will have to configure nodes herre
        else:
            rospy.loginfo("hier1")
            self.launcher.shutdown()
            self.diagnostics_pub.publish(
                create_diagnostic_message(
                    level=DiagnosticStatus.WARN,
                    name="[CTRL CTRL] Node Status",
                    message="Node shutting down.",
                )
            )
            return

        # Decides what to do based on the received state
        if state.scope == StateMachineScopeEnum.SLAM:
            self.state = state.cur_state

            if self.state == SLAMStatesEnum.FINISHED:
                rospy.set_param("/pure_pursuit/speed/target", 0.0)
                rospy.loginfo("hier2")
                set_state_finalized("pure_pursuit_control")

            elif (
                self.state == SLAMStatesEnum.EXPLORATION
                or self.state == SLAMStatesEnum.RACING
            ):
                if self.mission == AutonomousMission.TRACKDRIVE:
                    if self.state == SLAMStatesEnum.EXPLORATION:
                        self.launcher.launch_node(
                            "control_controller", f"launch/{self.mission}.launch"
                        )
                else:
                    self.launcher.launch_node(
                        "control_controller",
                        f"launch/{self.mission}_{self.state}.launch",
                    )
                    configure_node("pure_pursuit_control")
                if self.mission == AutonomousMission.TRACKDRIVE:
                    set_state_active("pure_pursuit_control")
                elif (
                    self.mission == AutonomousMission.SKIDPAD
                    and self.state == SLAMStatesEnum.RACING
                ):
                    set_state_active("pure_pursuit_control")
                elif (
                    self.mission == AutonomousMission.AUTOCROSS
                    and self.state == SLAMStatesEnum.EXPLORATION
                ):
                    set_state_active("pure_pursuit_control")

                elif (
                    self.mission == AutonomousMission.ACCELERATION
                    and self.state == SLAMStatesEnum.RACING
                ):
                    set_state_active("pure_pursuit_control")
                speed_target = 0.0
                if self.state == SLAMStatesEnum.EXPLORATION:
                    speed_target = 2.0 if self.car == "simulation" else 1.0

                elif self.state == SLAMStatesEnum.RACING:
                    if self.mission == AutonomousMission.TRACKDRIVE:
                        # Slow down for 3 seconds to let nodes start up
                        rospy.set_param("/pure_pursuit/speed/target", 0.3)

                        sleep(3)

                        speed_target = 10.0 if self.car == "simulation" else 5.0

                    elif self.mission == AutonomousMission.SKIDPAD:
                        speed_target = 10.0 if self.car == "simulation" else 2.0

                    elif self.mission == AutonomousMission.ACCELERATION:
                        speed_target = 10.0 if self.car == "simulation" else 3.0

                rospy.set_param("/pure_pursuit/speed/target", speed_target)

            else:
                self.launcher.shutdown()
                self.diagnostics_pub.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.WARN,
                        name="[CTRL CTRL] Node Status",
                        message="Node shutting down.",
                    )
                )


node = Controller()
