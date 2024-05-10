#! /usr/bin/python3
from collections import deque
from time import sleep

import rospy
from car_state import CarStateEnum
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from node_fixture import (
    AutonomousStatesEnum,
    SLAMStatesEnum,
    StateMachineScopeEnum,
    create_diagnostic_message,
)
from node_fixture.fixture import NodeManagingStatesEnum
from node_fixture.node_manager import NodeManager
from orion_autonomous_state import OrionAutonomousState
from pegasus_state import PegasusState
from simulation_state import SimulationState
from std_msgs.msg import Header
from ugr_msgs.msg import State


class AutonomousController(NodeManager):
    def __init__(self) -> None:
        """
        Autonomous controller

        Implemented according to T14/T15. Note that this class relies on an underlying model
        such as "simulation" or "pegasus". These models are the ones responsible for
        providing the data required to set the autonomous state.
        """
        self.car_name = rospy.get_param("car", "pegasus")
        if (
            self.car_name == "pegasus" or self.car_name == "simulation"
        ):  # when we use pegasus or simulation this node needs to be activated on its own
            super().__init__("autonomous_state", NodeManagingStatesEnum.ACTIVE)
        else:
            super().__init__("autonomous_state")
        self.spin()

    def doActivate(self):
        self.as_state = ""
        self.ccs = {}
        self.state_publisher = rospy.Publisher(
            "/state", State, queue_size=10, latch=True
        )
        self.as_state_publisher = rospy.Publisher(
            "/state/as", State, queue_size=10, latch=True
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        if self.car_name == "pegasus":
            self.car = PegasusState()
        elif self.car_name == "orion":
            self.car = OrionAutonomousState(self)
        elif self.car_name == "simulation":
            self.car = SimulationState()
        else:
            raise f"Unknown model! (model given was: '{self.car_name}')"

        self.mission_finished = False
        self.vehicle_stopped = True

        self.odom_avg = deque([], 100)

        # Setup
        while not self.configure_nodes():
            sleep(0.1)
            if rospy.is_shutdown():
                return

        rospy.Subscriber("/state", State, self.handle_external_state_change)
        rospy.Subscriber("/input/odom", Odometry, self.handle_odom)
        self.change_state(AutonomousStatesEnum.ASOFF)
        self.car.update(self.as_state)

    def active(self):
        """
        Main function to be executed in a loop.

        Follows flowchart in section T14.10 and related rules (T14/T15)

        - https://www.formulastudent.de/fileadmin/user_upload/all/2023/rules/FS-Rules_2023_v1.1.pdf
        - https://www.formulastudent.de/fileadmin/user_upload/all/2023/important_docs/FSG23_AS_Beginners_Guide_v1.1.pdf
        """
        # Gets car state as reported by our helper class (can be simulated or a specific car such as Peggy)
        self.ccs = self.car.get_state()
        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.ERROR
                if self.as_state == AutonomousStatesEnum.ASEMERGENCY
                else DiagnosticStatus.OK,
                "[GNRL] STATE: AS state",
                str(self.state),
            )
        )
        self.diagnostics_publisher.publish(
            create_diagnostic_message(
                DiagnosticStatus.OK, "[GNRL] STATE: Car state", str(self.ccs)
            )
        )

        # Monitor the nodes and enable EBS if required
        # ?? Not sure if the second part (after the 'and') is required, but it's there in the original code
        # ?? Basically disables the monitoring causing an emergency when the car is in ASOFF, so no where near ready to actually drive
        # ?? Must check when integrating on real car.
        if self.car_name == "pegasus" or self.car_name == "simulation":
            if self.get_health_level() == DiagnosticStatus.ERROR and not (
                self.as_state == AutonomousStatesEnum.ASOFF
                or self.as_state == AutonomousStatesEnum.ASFINISHED
            ):
                self.car.activate_EBS()
        else:
            if self.get_health_level() == DiagnosticStatus.ERROR:
                self.car.activate_EBS(0)

        if self.ccs["EBS"] == CarStateEnum.ACTIVATED:
            if self.mission_finished and self.vehicle_stopped:
                self.change_state(AutonomousStatesEnum.ASFINISHED)

            # ! This line is here to prevent rapid toggles between ASFINISHED and ASEMERGENCY as a result of self.vehicle_stopped rapidly switching
            # ! In a normal FS car this isn't a problem because you have to apply both EBS and the brakes in order to get the vehicle to a "standstill" state
            # ! But for pegasus (and currently simulation also) we can't really "apply the brakes"
            elif self.as_state != AutonomousStatesEnum.ASFINISHED:
                self.change_state(AutonomousStatesEnum.ASEMERGENCY)

        elif self.ccs["EBS"] == CarStateEnum.ON:
            if self.mission_finished and self.vehicle_stopped:
                if self.car_name == "pegasus" or self.car_name == "simulation":
                    self.car.activate_EBS()
                else:
                    self.car.activate_EBS(0)

            if (
                rospy.has_param("/mission")
                and rospy.get_param("/mission") != ""
                and self.ccs["ASMS"] == CarStateEnum.ON
                and (
                    self.ccs["ASB"] == CarStateEnum.ON
                    or self.ccs["ASB"] == CarStateEnum.ACTIVATED
                )
                and self.ccs["TS"] == CarStateEnum.ON
            ):
                if self.ccs["R2D"] == CarStateEnum.ACTIVATED:
                    self.change_state(AutonomousStatesEnum.ASDRIVE)

                else:
                    if (
                        self.ccs["ASB"] == CarStateEnum.ACTIVATED
                        and self.get_health_level() == DiagnosticStatus.OK
                    ):
                        self.change_state(AutonomousStatesEnum.ASREADY)
                    else:
                        self.change_state(AutonomousStatesEnum.ASOFF)
            else:
                self.change_state(AutonomousStatesEnum.ASOFF)

        self.car.update(self.as_state)

    def handle_odom(self, odom: Odometry):
        """
        Just keeps track of latest odometry estimate

        Args:
            odom: the odometry message containing speed information
        """

        # If vehicle is stopped and EBS is activated, vehicle will not be able to move, so
        # latch self.vehicle_stopped
        if self.vehicle_stopped and self.ccs["EBS"] == CarStateEnum.ACTIVATED:
            return

        self.odom_avg.append(abs(odom.twist.twist.linear.x))
        self.vehicle_stopped = sum(list(self.odom_avg)) / len(list(self.odom_avg)) < 0.1

    def change_state(self, new_state: AutonomousStatesEnum):
        """
        Actually changes state of this machine and publishes change

        Args:
            new_state: state to switch to.
        """

        if new_state == self.as_state:
            return

        self.activate_nodes(new_state, self.state)

        self.state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.AUTONOMOUS,
                prev_state=self.as_state,
                cur_state=new_state,
            )
        )
        self.as_state_publisher.publish(
            State(
                header=Header(stamp=rospy.Time.now()),
                scope=StateMachineScopeEnum.AUTONOMOUS,
                prev_state=self.as_state,
                cur_state=new_state,
            )
        )
        self.as_state = new_state

    def handle_external_state_change(self, state: State):
        """
        Handles state transition from other state machines

        Args:
            state: the state transition
        """

        if state.scope == StateMachineScopeEnum.SLAM:
            """
            When SLAM reports being in the finished mode, autonomous should perhaps
            also do something
            """

            self.mission_finished = state.cur_state == SLAMStatesEnum.FINISHED

        elif state.scope == StateMachineScopeEnum.AUTONOMOUS:
            return


node = AutonomousController()
