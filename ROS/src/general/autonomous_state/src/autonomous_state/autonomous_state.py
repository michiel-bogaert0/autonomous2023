#! /usr/bin/python3
import rospy
from ugr_msgs.msg import State
from nav_msgs.msg import Odometry
from node_fixture import AutonomousStatesEnum, StateMachineScopeEnum, SLAMStatesEnum, create_diagnostic_message
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from time import sleep
from collections import deque

from car_state import *


class AutonomousController:
    def __init__(self) -> None:
        """
        Autonomous controller

        Implemented according to T14/T15. Note that this class relies on an underlying model
        such as "simulation" or "pegasus". These models are the ones responsible for
        providing the data required to set the autonomous state.
        """
        rospy.init_node("as_controller")

        self.state = ""

        rospy.Subscriber("/state", State, self.handle_external_state_change)
        rospy.Subscriber("/input/odom", Odometry, self.handle_odom)

        self.state_publisher = rospy.Publisher("/state", State, queue_size=10, latch=True)
        self.diagnostics_publisher = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)

        self.car_name = rospy.get_param("car", "pegasus")

        if self.car_name == "pegasus":
            self.car = PegasusState()
        elif self.car_name == "simulation":
            self.car = SimulationState()
        else:
            raise f"Unkown model! (model given was: '{self.car_name}')"

        self.mission_finished = False
        self.vehicle_stopped = True

        self.odom_avg = deque([], 100)

        # Setup
        self.change_state(AutonomousStatesEnum.ASOFF)
        self.car.update(self.state)

        while not rospy.is_shutdown():
            self.main()

            self.car.update(self.state)
            sleep(0.05)

    def main(self):
        """
        Main function to be executed in a loop.

        Follows flowchart in section T14.10 and related rules (T14/T15)

        - https://www.formulastudent.de/fileadmin/user_upload/all/2023/rules/FS-Rules_2023_v1.1.pdf
        - https://www.formulastudent.de/fileadmin/user_upload/all/2023/important_docs/FSG23_AS_Beginners_Guide_v1.1.pdf
        """

        # Gets car state as reported by our helper class (can be simulated or a specific car such as Peggy)
        ccs = self.car.get_state()

        self.diagnostics_publisher.publish(create_diagnostic_message(DiagnosticStatus.ERROR if self.state == AutonomousStatesEnum.ASEMERGENCY else DiagnosticStatus.OK, "[GNRL] STATE: AS state", str(self.state)))
        self.diagnostics_publisher.publish(create_diagnostic_message(DiagnosticStatus.OK, "[GNRL] STATE: Car state", str(ccs)))

        if ccs["EBS"] == carStateEnum.ACTIVATED

            if self.mission_finished and self.vehicle_stopped:
                self.change_state(AutonomousStatesEnum.ASFINISHED)
            else:
                self.change_state(AutonomousStatesEnum.ASEMERGENCY)

        elif ccs["EBS"] == carStateEnum.ON:

            if self.mission_finished and self.vehicle_stopped:
                self.car.activate_EBS()

            if (
                rospy.has_param("/mission") and rospy.get_param("/mission") != ""
                and ccs["ASMS"] == carStateEnum.ON
                and (
                    ccs["ASB"] == carStateEnum.ON
                    or ccs["ASB"] == carStateEnum.ACTIVATED
                )
                and ccs["TS"] == carStateEnum.ON
            ):
                if ccs["R2D"] == carStateEnum.ACTIVATED:

                    self.change_state(AutonomousStatesEnum.ASDRIVE)

                else:

                    if ccs["ASB"] == carStateEnum.ACTIVATED:
                        self.change_state(AutonomousStatesEnum.ASREADY)
                    else:
                        self.change_state(AutonomousStatesEnum.ASOFF)
            else:
                self.change_state(AutonomousStatesEnum.ASOFF)

    def handle_odom(self, odom: Odometry):
        """
        Just keeps track of latest odometry estimate

        Args:
            odom: the odometry message containing speed information
        """
        self.odom_avg.append(abs(odom.twist.twist.linear.x))
        self.vehicle_stopped = sum(list(self.odom_avg)) / len(list(self.odom_avg)) < 0.1

    def change_state(self, new_state: AutonomousStatesEnum):
        """
        Actually changes state of this machine and publishes change

        Ags:
            new_state: state to switch to.
        """

        if new_state == self.state:
            return

        self.state_publisher.publish(
            State(
                scope=StateMachineScopeEnum.AUTONOMOUS,
                prev_state=self.state,
                cur_state=new_state,
            )
        )
        self.state = new_state

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

            self.mission_finished = state.cur_state == SLAMStatesEnum.FINISHED or state.cur_state == SLAMStatesEnum.FINISHING

        elif state.scope == StateMachineScopeEnum.AUTONOMOUS:
            return


node = AutonomousController()
