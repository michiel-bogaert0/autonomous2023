#! /usr/bin/python3
from time import sleep

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from node_fixture.fixture import (
    AutonomousMission,
    AutonomousStatesEnum,
    SLAMStatesEnum,
    StateMachineScopeEnum,
    create_diagnostic_message,
)
from node_fixture.node_management import (
    configure_node,
    load_params,
    set_state_active,
    set_state_finalized,
    set_state_inactive,
)
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

        self.mission = ""
        self.car = rospy.get_param("/car")

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

                # Configure parameters after mission is set
                load_params(self.mission)
                # Confige nodes after mission is set
                # SLAM
                configure_node("slam_mcl")
                configure_node("fastslam")
                configure_node("loopclosure")
                configure_node("map_publisher")
                # Control
                configure_node("pure_pursuit_control")
                configure_node("MPC_tracking_control")
                configure_node("control_path_publisher")
                configure_node("pathplanning")
                configure_node("boundary_estimation")

                # Reset loop counter
                rospy.ServiceProxy("/reset_closure", Empty)

                if self.mission == AutonomousMission.ACCELERATION:
                    self.target_lap_count = 1
                    new_state = SLAMStatesEnum.RACING
                    set_state_active("slam_mcl")
                    set_state_active("loopclosure")
                    set_state_active("map_publisher")

                    set_state_active("pure_pursuit_control")
                    # set_state_active("MPC_tracking_control")
                    set_state_active("control_path_publisher")
                elif self.mission == AutonomousMission.SKIDPAD:
                    self.target_lap_count = 1
                    new_state = SLAMStatesEnum.RACING
                    set_state_active("slam_mcl")
                    set_state_active("loopclosure")
                    set_state_active("map_publisher")

                    set_state_active("pure_pursuit_control")
                    # set_state_active("MPC_tracking_control")
                    set_state_active("control_path_publisher")
                elif self.mission == AutonomousMission.AUTOCROSS:
                    self.target_lap_count = 1
                    new_state = SLAMStatesEnum.EXPLORATION
                    set_state_active("fastslam")
                    set_state_active("loopclosure")

                    # set_state_active("pure_pursuit_control")
                    set_state_active("MPC_tracking_control")
                    set_state_active("pathplanning")
                    set_state_active("boundary_estimation")
                elif self.mission == AutonomousMission.TRACKDRIVE:
                    self.target_lap_count = 10
                    new_state = SLAMStatesEnum.EXPLORATION
                    set_state_active("fastslam")
                    set_state_active("loopclosure")

                    # set_state_active("pure_pursuit_control")
                    set_state_active("MPC_tracking_control")
                    set_state_active("pathplanning")
                    set_state_active("boundary_estimation")
                else:
                    self.target_lap_count = -1
                    new_state = SLAMStatesEnum.EXPLORATION
                    set_state_inactive("fastslam")
                    set_state_inactive("slam_mcl")
                    set_state_inactive("loopclosure")
                    set_state_inactive("map_publisher")

                    set_state_inactive("pure_pursuit_control")
                    set_state_inactive("MPC_tracking_control")
                    set_state_inactive("control_path_publisher")
                    set_state_inactive("pathplanning")
                    set_state_inactive("boundary_estimation")
            else:
                set_state_finalized("slam_mcl")
                set_state_finalized("fastslam")
                set_state_finalized("loopclosure")
                set_state_finalized("map_publisher")

                set_state_finalized("pure_pursuit_control")
                set_state_finalized("MPC_tracking_control")
                set_state_finalized("control_path_publisher")
                set_state_finalized("pathplanning")
                set_state_finalized("boundary_estimation")
                rospy.set_param("/pure_pursuit/speed/target", 0.0)
                rospy.set_param("/mpc/speed/target", 0.0)

        elif not rospy.has_param("/mission"):
            set_state_finalized("slam_mcl")
            set_state_finalized("fastslam")
            set_state_finalized("loopclosure")
            set_state_finalized("map_publisher")

            set_state_finalized("pure_pursuit_control")
            set_state_finalized("MPC_tracking_control")
            set_state_finalized("control_path_publisher")
            set_state_finalized("pathplanning")
            set_state_finalized("boundary_estimation")
            rospy.set_param("/pure_pursuit/speed/target", 0.0)
            rospy.set_param("/mpc/speed/target", 0.0)
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

        # If we did enough laps, switch to finished
        if self.target_lap_count <= laps.data:
            new_state = SLAMStatesEnum.FINISHED
            rospy.set_param("/pure_pursuit/speed/target", 0.0)
            rospy.set_param("/mpc/speed/target", 0.0)
            self.change_state(new_state)
            return

        # If we did one lap in trackdrive and exploration, switch to racing
        if (
            self.mission == AutonomousMission.TRACKDRIVE
            and self.state == SLAMStatesEnum.EXPLORATION
        ):
            rospy.loginfo("Exploration finished, switching to racing")
            new_state = SLAMStatesEnum.RACING
            set_state_active("slam_mcl")
            speed_target = 10.0 if self.car == "simulation" else 5.0
            rospy.set_param("/pure_pursuit/speed/target", speed_target)
            rospy.set_param("/mpc/speed/target", speed_target)
            sleep(0.5)
            set_state_inactive("fastslam")
            set_state_inactive("pathplanning")
            self.change_state(new_state)


node = Controller()
