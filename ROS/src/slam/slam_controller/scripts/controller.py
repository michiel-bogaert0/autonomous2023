#! /usr/bin/python3
import rospy
from enum import Enum
from std_msgs.msg import UInt16
from std_srvs.srv import Empty
from ugr_msgs.msg import State
from node_launcher import NodeLauncher


class AutonomousState(str, Enum):
    IDLE = 'idle'
    EXPLORATION = 'exploration'
    RACING = 'racing'
    FINISHED = 'finished'


class AutonomousMission(str, Enum):
    AUTOCROSS = 'autocross',
    ACCELERATION = 'acceleration',
    TRACKDRIVE = 'trackdrive',


class Controller:

    def __init__(self) -> None:
        rospy.init_node("slam_controller")

        # Loop closure for state change detection
        self.state_publisher = rospy.Subscriber(
            "/input/loopclosure", UInt16, self.lapFinished)

        self.state = AutonomousState.IDLE

        self.launcher = NodeLauncher()
        self.mission = ""

        self.target_lap_count = -1

        rospy.Subscriber("/state", State, self.handle_state_change)
        rospy.Publisher("/state", State, queue_size=10)

        while not rospy.is_shutdown():
            self.launcher.run()
            rospy.sleep(0.01)

    def handle_state_change(self, state: State):

        new_state = self.state

        if state.scope == "ugentracing":

            if (self.state == AutonomousState.IDLE or self.state == AutonomousState.FINISHED) and state.cur_state == "ASDrive":

                if rospy.has_param("/mission"):
                    # Go to state depending on mission
                    self.mission = rospy.get_param("/mission")

                    # Reset loop counter
                    rospy.ServiceProxy('/reset_closure', Empty)

                    if self.mission == AutonomousMission.ACCELERATION:
                        self.target_lap_count = 1
                        new_state = AutonomousState.RACING
                    else:
                        new_state = AutonomousState.EXPLORATION
                        if self.mission == AutonomousMission.AUTOCROSS:
                            self.target_lap_count = 1
                        elif self.mission == AutonomousMission.TRACKDRIVE:
                            self.target_lap_count = 10
                        else:
                            self.target_lap_count = -1  # idk, no lap count I guess

                    # Launch nodes
                    self.launcher.launch_node(
                        "slam_controller", f"launch/{self.mission}_{new_state}.launch")

            elif state.cur_state != "ASDrive":
                # Just stop everything
                self.launcher.shutdown()

        self.state_publisher.publish(
            State(scope="autonomous", prev_state=self.state, cur_state=new_state))
        self.state = new_state

    def lapFinished(self, laps):

        if self.target_lap_count == laps:

            if self.state == AutonomousState.EXPLORATION:

                if self.mission == AutonomousMission.TRACKDRIVE:
                    self.state = AutonomousState.RACING

                    # Relaunch (different) nodes
                    self.launcher.launch_node(
                        "slam_controller", f"launch/{self.mission}_{self.state}.launch")
                else:
                    self.state = AutonomousState.FINISHED
                    self.launcher.shutdown()
            else:
                self.state = AutonomousState.FINISHED
                self.launcher.shutdown()


node = Controller()
