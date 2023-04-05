#! /usr/bin/python3
import rospy
from enum import Enum
from std_msgs.msg import UInt16
from std_srvs.srv import Empty
from ugr_msgs.msg import State
from node_launcher import NodeLauncher
from nav_msgs.msg import Odometry
from node_fixture import (
    AutonomousStatesEnum,
    StateMachineScopeEnum,
)
import rosparam

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

        self.state = AutonomousStatesEnum.ASOFF

        rospy.Subscriber("/state", State, self.handle_external_state_change)
        rospy.Subscriber("/input/can", UInt16, self.handle_can)
        rospy.Subscriver("/input/odom", Odometry, self.handle_odom)

        self.state_publisher = rospy.Publisher("/state", State, queue_size=10)

        self.car_name = rospy.get_param("~model", "pegasus")

        if self.car_name == "pegasus":
            self.car = PegasusState()
        else:
            raise f"Unkown model! (model given was: '{self.car_name}')"

        # Setup
        self.car.set_state(
            {
                "TS": carStateEnum.OFF,
                "R2D": carStateEnum.OFF,
                "ASB": carStateEnum.OFF,
                "EBS": carStateEnum.OFF,
                "ASSI": carStateEnum.OFF,
            }
        )

        self.mission_finished = False
        self.vehicle_stopped = False

        while not rospy.is_shutdown():
            self.main()
            self.car.update(self.state)
            rospy.sleep(0.1)

    def main(self):
        """
        Main function to be executed in a loop.
        
        Follows flowchart in section T14.10 and related rules (T14/T15)
        
        - https://www.formulastudent.de/fileadmin/user_upload/all/2023/rules/FS-Rules_2023_v1.1.pdf
        - https://www.formulastudent.de/fileadmin/user_upload/all/2023/important_docs/FSG23_AS_Beginners_Guide_v1.1.pdf
        """

        # Gets car state as reported by our helper class (can be simulated or a specific car such as Peggy)
        ccs = self.car.get_state()

        if ccs["EBS"] == carStateEnum.ACTIVATED:

            if self.mission_finished and self.vehicle_stopped:
                self.change_state(AutonomousStatesEnum.ASFINISHED)
            else:
                self.change_state(AutonomousStatesEnum.ASEMERGENCY)

        elif ccs["EBS"] == carStateEnum.ON:

            if rosparam.has_param("/mission") and ccs["ASMS"] == carStateEnum.ON and (ccs["ASB"] == carStateEnum.ON or ccs["ASB"] == carStateEnum.ACTIVATED) and ccs["TS"] == carStateEnum.ON:
                if ccs["R2D"] == carStateEnum.ACTIVATED:

                    self.change_state(AutonomousStatesEnum.ASDRIVE)

                else:

                    if ccs["ASB"] == carStateEnum.ACTIVATED:
                        self.change_state(AutonomousStatesEnum.ASREADY)
                    else:
                        self.change_state(AutonomousStatesEnum.ASOFF)

    def handle_odom(self, odom: Odometry):
        """
        Just keeps track of latest odometry estimate

        Args:
            odom: the odometry message containing speed information
        """

        self.vehicle_stopped = abs(odom.twist.twist.linear.x) < 0.05

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

            self.mission_finished = self.state == AutonomousStatesEnum.ASDRIVE

        elif state.scope == StateMachineScopeEnum.AUTONOMOUS:
            return

node = AutonomousController()
