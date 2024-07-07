#! /usr/bin/python3
from threading import Thread

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from node_fixture.fixture import (
    AutonomousMission,
    AutonomousStatesEnum,
    SLAMStatesEnum,
    StateMachineScopeEnum,
    create_diagnostic_message,
)
from node_fixture.node_manager import NodeManager, load_params
from std_msgs.msg import Float32, Header, UInt16
from std_srvs.srv import Empty
from ugr_msgs.msg import State


class Controller(NodeManager):
    def __init__(self) -> None:
        """
        SLAM controller
        """

        super().__init__("slam_controller")

        self.slam_state = None

        self.mission = ""
        self.car = rospy.get_param("/car")

        self.target_lap_count = -1

        self.change_mission_thread = Thread(target=self.change_mission)

        rospy.Subscriber("/state", State, self.handle_state_change)
        rospy.Subscriber("/input/lapComplete", UInt16, self.lapFinished)

        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.state_publisher = rospy.Publisher(
            "/state", State, queue_size=10, latch=True
        )
        self.slam_state_publisher = rospy.Publisher(
            "/state/slam", State, queue_size=10, latch=True
        )

        self.brake_publisher = rospy.Publisher(
            "/iologik/input1", Float32, queue_size=10
        )

        self.change_state(SLAMStatesEnum.IDLE)

        self.spin()

    def change_mission(self):
        """
        Changes the mission of the car
        """
        # Go to state depending on mission
        self.mission = rospy.get_param("/mission")

        # Configure parameters after mission is set. Also loads in the parameter for the node manager
        load_params(self.mission)

        # Configure nodes after mission is set
        # When this doesn't work, the thread joins, so the mission change is not executed
        # But the health state will reflect that and the state machine will try again next loop iteration
        if not self.configure_nodes():
            return

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
            self.target_lap_count = 10
            new_state = SLAMStatesEnum.EXPLORATION
        elif self.mission == AutonomousMission.DVSV:
            self.target_lap_count = 1
            new_state = SLAMStatesEnum.RACING
        elif self.mission == AutonomousMission.INPSPECTION:
            self.target_lap_count = 1
            new_state = SLAMStatesEnum.RACING
        else:
            self.target_lap_count = -1
            new_state = SLAMStatesEnum.EXPLORATION

        # Same logic as in configure nodes
        if not self.activate_nodes(new_state, self.slam_state):
            return

        # Only if that worked, change state
        self.change_state(new_state)

    def active(self):
        """
        Updates the internal state and launches or kills nodes if needed
        """

        new_state = self.slam_state

        if self.slam_state == SLAMStatesEnum.IDLE or (
            rospy.has_param("/mission") and rospy.get_param("/mission") != self.mission
        ):
            if rospy.has_param("/mission") and rospy.get_param("/mission") != "":
                if not self.change_mission_thread.is_alive():
                    # You cannot 'restart' a thread, so we make a new one after the other one is not alive anymore
                    self.change_mission_thread = Thread(target=self.change_mission)
                    self.change_mission_thread.start()

            else:
                self.unconfigure_nodes()
                rospy.set_param("/speed/target", 0.0)

        elif not rospy.has_param("/mission"):
            self.unconfigure_nodes()

            rospy.set_param("/speed/target", 0.0)
            new_state = SLAMStatesEnum.IDLE

        self.change_state(new_state)

        # Diagnostics
        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.OK, "[GNRL] STATE: SLAM state", str(self.slam_state)
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

        new_state = self.slam_state

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

        if new_state == self.slam_state:
            return

        self.state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.SLAM,
                prev_state=self.slam_state,
                cur_state=new_state,
            )
        )
        self.slam_state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.SLAM,
                prev_state=self.slam_state,
                cur_state=new_state,
            )
        )
        self.slam_state = new_state

    def lapFinished(self, laps):
        """
        Subscriber callback for the lap counter. Does an internal state transition if required

        Args:
            laps: the UInt16 message containing the lap count
        """

        # If we did enough laps, switch to finished
        if self.target_lap_count <= laps.data:
            new_state = SLAMStatesEnum.FINISHED
            rospy.set_param("/speed/target", 0.0)
            self.brake_publisher.publish(Float32(data=20.0))
            self.change_state(new_state)
            return

        # If we did one lap in trackdrive and exploration, switch to racing
        if (
            self.mission == AutonomousMission.TRACKDRIVE
            and self.slam_state == SLAMStatesEnum.EXPLORATION
        ):
            rospy.loginfo("Exploration finished, switching to racing")
            new_state = SLAMStatesEnum.RACING

            # Only change if the nodes are activated succesfully
            if self.activate_nodes(new_state, self.slam_state):
                speed_target = rospy.get_param("/speed/target_racing")
                rospy.set_param("/speed/target", speed_target)

                self.change_state(new_state)


node = Controller()

# Comes here when rospy.is_shutdown() == True (breaks out of spin())
# So make sure to join the mission thread
if node.change_mission_thread.is_alive():
    node.change_mission_thread.join()
