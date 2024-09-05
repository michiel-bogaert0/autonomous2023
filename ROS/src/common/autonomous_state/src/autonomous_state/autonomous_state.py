#! /usr/bin/python3
from collections import deque
from time import sleep

import rospy
from car_state import CarStateEnum
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
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

        super().__init__("autonomous_state", NodeManagingStatesEnum.ACTIVE)

        self.as_state = ""

        rospy.Subscriber("/state", State, self.handle_external_state_change)
        rospy.Subscriber("/input/odom", Odometry, self.handle_odom)

        self.state_publisher = rospy.Publisher(
            "/state", State, queue_size=10, latch=True
        )
        self.as_state_publisher = rospy.Publisher(
            "/state/as", State, queue_size=10, latch=True
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        self.car_name = rospy.get_param("car", "pegasus")

        if self.car_name == "pegasus":
            self.car = PegasusState()
        elif self.car_name == "simulation":
            self.car = SimulationState()
        else:
            raise f"unknown model! (model given was: '{self.car_name}')"

        self.mission_finished = False
        self.vehicle_stopped = True

        self.odom_avg = deque([], 100)

        # Setup
        while not self.configure_nodes():
            sleep(0.1)

            if rospy.is_shutdown():
                return

        self.switch_controllers()

        self.change_state(AutonomousStatesEnum.ASOFF)
        self.car.update(self.as_state)

        self.spin()

    def switch_controllers(self):
        rospy.wait_for_service("/ugr/car/controller_manager/switch_controller")
        try:
            switch_controller = rospy.ServiceProxy(
                "/ugr/car/controller_manager/switch_controller", SwitchController
            )

            req = SwitchControllerRequest()
            req.start_controllers = [
                "joint_state_controller",
                "steering_position_controller",
                "drive_velocity_controller",
            ]
            req.stop_controllers = []
            req.strictness = SwitchControllerRequest.BEST_EFFORT

            response = switch_controller(req)

            if not response.ok:
                rospy.logerr("Could not start controllers")
            else:
                rospy.loginfo("Controllers started")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def active(self):
        """
        Main function to be executed in a loop.

        Follows flowchart in section T14.10 and related rules (T14/T15)

        - https://www.formulastudent.de/fileadmin/user_upload/all/2023/rules/FS-Rules_2023_v1.1.pdf
        - https://www.formulastudent.de/fileadmin/user_upload/all/2023/important_docs/FSG23_AS_Beginners_Guide_v1.1.pdf
        """

        # Gets car state as reported by our helper class (can be simulated or a specific car such as Peggy)
        ccs = self.car.get_state()

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
                DiagnosticStatus.OK, "[GNRL] STATE: Car state", str(ccs)
            )
        )

        # Monitor the nodes and enable EBS if required
        # ?? Not sure if the second part (after the 'and') is required, but it's there in the original code
        # ?? Basically disables the monitoring causing an emergency when the car is in ASOFF, so no where near ready to actually drive
        # ?? Must check when integrating on real car.
        if self.get_health_level() == DiagnosticStatus.ERROR and not (
            self.as_state == AutonomousStatesEnum.ASOFF
            or self.as_state == AutonomousStatesEnum.ASFINISHED
        ):
            self.car.activate_EBS()

        if ccs["EBS"] == CarStateEnum.ACTIVATED:
            if self.mission_finished and self.vehicle_stopped:
                self.change_state(AutonomousStatesEnum.ASFINISHED)

            # ! This line is here to prevent rapid toggles between ASFINISHED and ASEMERGENCY as a result of self.vehicle_stopped rapidly switching
            # ! In a normal FS car this isn't a problem because you have to apply both EBS and the brakes in order to get the vehicle to a "standstill" state
            # ! But for pegasus (and currently simulation also) we can't really "apply the brakes"
            elif self.as_state != AutonomousStatesEnum.ASFINISHED:
                self.change_state(AutonomousStatesEnum.ASEMERGENCY)

        elif ccs["EBS"] == CarStateEnum.ON:
            if self.mission_finished and self.vehicle_stopped:
                self.car.activate_EBS()

            if (
                rospy.has_param("/mission")
                and rospy.get_param("/mission") != ""
                and ccs["ASMS"] == CarStateEnum.ON
                and (
                    ccs["ASB"] == CarStateEnum.ON
                    or ccs["ASB"] == CarStateEnum.ACTIVATED
                )
                and ccs["TS"] == CarStateEnum.ON
            ):
                if ccs["R2D"] == CarStateEnum.ACTIVATED:
                    self.change_state(AutonomousStatesEnum.ASDRIVE)

                else:
                    if (
                        ccs["ASB"] == CarStateEnum.ACTIVATED
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
        if (
            self.vehicle_stopped
            and self.car.get_state()["EBS"] == CarStateEnum.ACTIVATED
        ):
            return

        self.odom_avg.append(abs(odom.twist.twist.linear.x))
        self.vehicle_stopped = sum(list(self.odom_avg)) / len(list(self.odom_avg)) < 0.1

    def change_state(self, new_state: AutonomousStatesEnum):
        """
        Actually changes state of this machine and publishes change

        Ags:
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
