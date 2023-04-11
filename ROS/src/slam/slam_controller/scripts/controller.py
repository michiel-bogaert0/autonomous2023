#! /usr/bin/python3
import rospy
from enum import Enum
from std_msgs.msg import UInt16
from std_srvs.srv import Empty
from ugr_msgs.msg import State
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from node_launcher import NodeLauncher
from node_fixture.node_fixture import AutonomousMission, SLAMStatesEnum, AutonomousStatesEnum, StateMachineScopeEnum, create_diagnostic_message



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
        
        self.state_publisher = rospy.Publisher("/state", State, queue_size=10, latch=True)
        self.diagnostics = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)

        while not rospy.is_shutdown():
            self.launcher.run()
            rospy.sleep(0.01)

    def handle_state_change(self, state: State):
        """
        Handles state transition from other state machines

        Args:
            state: the state transition
        """

        new_state = self.state

        if state.scope == StateMachineScopeEnum.AUTONOMOUS:

            if (
                self.state == SLAMStatesEnum.IDLE
                or self.state == SLAMStatesEnum.FINISHED
            ) and state.cur_state == AutonomousStatesEnum.ASDRIVE:

                if rospy.has_param("/mission"):
                    # Go to state depending on mission
                    self.mission = rospy.get_param("/mission")

                    # Reset loop counter
                    rospy.ServiceProxy("/reset_closure", Empty)

                    if self.mission == AutonomousMission.ACCELERATION:
                        self.target_lap_count = 1
                        new_state = SLAMStatesEnum.RACING
                    else:
                        new_state = SLAMStatesEnum.EXPLORATION
                        self.target_lap_count = 1
 
                    # Launch nodes
                    self.launcher.launch_node(
                        "slam_controller", f"launch/{self.mission}_{new_state}.launch"
                    )
                    
                    # Publish mission
                    self.diagnostics.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name="[SLAM Controller] new mission",
                            message=f"{self.mission}",
                        )
                    )

            elif state.cur_state != AutonomousStatesEnum.ASDRIVE:
                # Just stop everything
                self.launcher.shutdown()
                new_state = SLAMStatesEnum.IDLE

        elif state.scope == StateMachineScopeEnum.SLAM:
            return

        self.state_publisher.publish(
            StateState(scope=StateMachineScopeEnum.SLAM, prev_state=self.state, cur_state=new_state)
        )
        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[SLAM Controller] new state",
                message=f"{new_state}",
            )
        )
        self.state = new_state

    def lapFinished(self, laps):
        """
        Subscriber callback for the lap counter. Does an internal state transition if required

        Args:
            laps: the UInt16 message containing the lap count
        """

        if self.target_lap_count == laps.data:

            new_state = self.state

            if self.state == SLAMStatesEnum.EXPLORATION:

                if self.mission == AutonomousMission.TRACKDRIVE:
                    new_state = SLAMStatesEnum.RACING
                    self.diagnostics.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name="[SLAM Controller] new state",
                            message=f"{new_state}",
                        )
                    )

                    self.target_lap_count = 10

                    # Relaunch (different) nodes
                    self.launcher.launch_node(
                        "slam_controller", f"launch/{self.mission}_{new_state}.launch"
                    )
                else:
                    new_state = SLAMStatesEnum.FINISHED
                    # Publish diagnostic finished state
                    self.diagnostics.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name="[SLAM Controller] new state",
                            message=f"{new_state}",
                        )
                    )
                    self.launcher.shutdown()
            else:
                new_state = SLAMStatesEnum.FINISHED
                # Publish diagnostic finished state
                self.diagnostics.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.OK,
                        name="[SLAM Controller] new state",
                        message=f"{new_state}",
                    )
                )
                self.launcher.shutdown()

            self.state_publisher.publish(
                State(scope=StateMachineScopeEnum.SLAM, prev_state=self.state, cur_state=new_state)
            )
            self.state = new_state


node = Controller()
