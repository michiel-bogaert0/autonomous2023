#! /usr/bin/python3
from time import sleep

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import (
    AutonomousMission,
    AutonomousStatesEnum,
    SLAMStatesEnum,
    StateMachineScopeEnum,
    create_diagnostic_message,
)
from node_launcher.node_launcher import NodeLauncher
from std_msgs.msg import Header, UInt16
from std_srvs.srv import Empty
from ugr_msgs.msg import State


class Controller:
    def __init__(self) -> None:
        """
        SLAM controller
        """
        rospy.init_node("slam_controller")

        self.state = SLAMStatesEnum.IDLE

        self.launcher = NodeLauncher()
        self.mission = ""

        self.target_lap_count = -1

        rospy.Subscriber("/state", State, self.handle_state_change)
        rospy.Subscriber("/input/loopclosure", UInt16, self.lapFinished)

        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.state_publisher = rospy.Publisher(
            "/state", State, queue_size=10, latch=True
        )

        while not rospy.is_shutdown():
            try:
                self.launcher.run()
                self.diagnostics_publisher.publish(
                    create_diagnostic_message(
                        DiagnosticStatus.OK, "[SLAM] Node launching", ""
                    )
                )
            except Exception as e:
                self.diagnostics_publisher.publish(
                    create_diagnostic_message(
                        DiagnosticStatus.ERROR, "[SLAM] Node launching", str(e)
                    )
                )
            self.update()

            sleep(0.1)

    def update(self):
        """
        Updates the internal state and launches or kills nodes if needed
        """
        new_state = self.state
        if self.state == SLAMStatesEnum.IDLE or (
            rospy.has_param("/mission") and rospy.get_param("/mission") != self.mission
        ):
            if rospy.has_param("/mission") and rospy.get_param("/mission") != "":
                # Go to state depending on mission
                self.mission = rospy.get_param("/mission")
                # Reset loop counter
                rospy.ServiceProxy("/reset_closure", Empty)

                if self.mission == AutonomousMission.ACCELERATION:
                    self.target_lap_count = 1
                    new_state = SLAMStatesEnum.RACING
                elif self.mission == AutonomousMission.SKIDPAD:
                    self.target_lap_count = 1
                    new_state = SLAMStatesEnum.RACING
                elif self.mission == AutonomousMission.AUTOCROSS:
                    self.target_lap_count = 1
                    new_state = SLAMStatesEnum.EXPLORATION
                elif self.mission == AutonomousMission.TRACKDRIVE:
                    self.target_lap_count = 1
                    new_state = SLAMStatesEnum.EXPLORATION
                else:
                    self.target_lap_count = -1
                    new_state = SLAMStatesEnum.EXPLORATION

                self.launcher.launch_node(
                    "slam_controller", f"launch/{self.mission}_{new_state}.launch"
                )
            else:
                self.launcher.shutdown()
        elif not rospy.has_param("/mission"):
            self.launcher.shutdown()
            new_state = SLAMStatesEnum.IDLE

        self.change_state(new_state)

        # Diagnostics
        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.OK, "[GNRL] STATE: SLAM state", str(self.state)
            )
        )
        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.OK, "[GNRL] Lap target", str(self.target_lap_count)
            )
        )
        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.OK, "[GNRL] MISSION", str(self.mission)
            )
        )

    def handle_state_change(self, state: State):
        """
        Handles state transition from other state machines

        Args:
            state: the state transition
        """

        new_state = self.state

        if state.scope == StateMachineScopeEnum.AUTONOMOUS:
            if state.cur_state == AutonomousStatesEnum.ASREADY:
                new_state = SLAMStatesEnum.IDLE

        self.change_state(new_state)

    def change_state(self, new_state: SLAMStatesEnum):
        """
        Actually changes state of this machine and publishes change

        Args:
            new_state: state to switch to.
        """

        if new_state == self.state:
            return

        self.state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.SLAM,
                prev_state=self.state,
                cur_state=new_state,
            )
        )
        self.state = new_state

    def lapFinished(self, laps):
        """
        Subscriber callback for the lap counter. Does an internal state transition if required

        Args:
            laps: the UInt16 message containing the lap count
        """

        if self.target_lap_count <= laps.data:
            new_state = self.state

            if self.state == SLAMStatesEnum.EXPLORATION:
                if self.mission == AutonomousMission.TRACKDRIVE:
                    new_state = SLAMStatesEnum.RACING

                    self.target_lap_count = 10

                    # Relaunch (different) nodes
                    self.launcher.launch_node(
                        "slam_controller", f"launch/{self.mission}_{new_state}.launch"
                    )
                else:
                    new_state = SLAMStatesEnum.FINISHED
            else:
                new_state = SLAMStatesEnum.FINISHED

            self.change_state(new_state)


node = Controller()
