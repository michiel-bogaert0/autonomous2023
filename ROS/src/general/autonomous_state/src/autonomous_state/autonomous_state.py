#! /usr/bin/python3
import rospy
from enum import Enum
from std_msgs.msg import UInt16
from std_srvs.srv import Empty
from ugr_msgs.msg import State
from node_launcher import NodeLauncher
from nav_msgs.msg import Odometry
from node_fixture import (
    AutonomousMission,
    SLAMStatesEnum,
    AutonomousStatesEnum,
    StateMachineScopeEnum,
)
import rosparam

from car_state import *


class AutonomousController:
    def __init__(self) -> None:
        """
        Autonomous controller
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
                "SA": carStateEnum.UNAVAILABLE,
                "ASB": carStateEnum.UNAVAILABLE,
                "EBS": carStateEnum.UNAVAILABLE,
                "ASSI": carStateEnum.OFF,
            }
        )

        self.t = rospy.Time().to_sec()

        while not rospy.is_shutdown():
            self.main()
            rospy.sleep(0.01)

    def main(self):

        ccs = self.car.get_state()

        # Follows flowchart in figure 17, section T14.10 in rules
        # "Autonomous System Status Definitions"

        if (
            self.state == AutonomousStatesEnum.ASOFF
            and ccs["EBS"] == carStateEnum.ARMED
            and ccs["TS"] == carStateEnum.ON
            and ccs["ASMS"] == carStateEnum.ON
        ):
            if rosparam.has_param("/mission"):

                if rosparam.get_param("/mission") == AutonomousMission.MANUAL:
                    self.change_state(AutonomousStatesEnum.MANUAL)
                    self.car.set_state(
                        {
                            "TS": carStateEnum.ON,
                            "R2D": carStateEnum.ON,
                            "SA": carStateEnum.UNAVAILABLE,
                            "ASB": carStateEnum.UNAVAILABLE,
                            "EBS": carStateEnum.UNAVAILABLE,
                            "ASSI": carStateEnum.OFF,
                        }
                    )

                else:
                    self.t = rospy.Time().to_sec()
                    self.change_state(AutonomousStatesEnum.ASREADY)
                    self.car.set_state(
                        {
                            "TS": carStateEnum.ON,
                            "R2D": carStateEnum.OFF,
                            "SA": carStateEnum.AVAILABLE,
                            "ASB": carStateEnum.ENGAGED,
                            "EBS": carStateEnum.ARMED,
                            "ASSI": carStateEnum.YELLOW_CONTINUOUS,
                        }
                    )

        elif self.state == AutonomousStatesEnum.ASREADY:

            if rospy.Time().to_sec() - self.t > 5 and ccs["GO"] == carStateEnum.ON:
                self.change_state(AutonomousStatesEnum.ASDRIVE)
                self.car.set_state(
                    {
                        "TS": carStateEnum.ON,
                        "R2D": carStateEnum.ON,
                        "SA": carStateEnum.AVAILABLE,
                        "ASB": carStateEnum.AVAILABLE,
                        "EBS": carStateEnum.ARMED,
                        "ASSI": carStateEnum.YELLOW_FLASH,
                    }
                )

            # if self.state == AutonomousStatesEnum.ASREADY: # And other stuff
            # self.change_state(AutonomousStatesEnum.ASOFF)

    def handle_odom(self, odom: Odometry):
        """
        Checks if we are currently in ASFINISHING and if speed becomes almost zero, we do a state change to ASFINISHED

        Args:
            odom: the odometry message containing speed information
        """

        if self.state == AutonomousStatesEnum.ASFINISHING:
            # Check if our speed is almost zero
            if odom.twist.twist.linear.x < 0.1:
                self.change_state(AutonomousStatesEnum.ASFINISHED)
                self.car.set_state(
                    {
                        "TS": carStateEnum.OFF,
                        "R2D": carStateEnum.OFF,
                        "SA": carStateEnum.UNAVAILABLE,
                        "EBS": carStateEnum.ACTIVATED,
                        "ASSI": carStateEnum.BLUE_CONTINUOUS,
                    }
                )

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

        new_state = self.state

        if state.scope == StateMachineScopeEnum.SLAM:
            """
            When SLAM reports being in the finished mode, autonomous should
            also do something
            """

            if (
                self.state == AutonomousStatesEnum.ASDRIVE
                and state.cur_state == SLAMStatesEnum.FINISHED
            ):
                new_state = AutonomousStatesEnum.ASFINISHING

        elif state.scope == StateMachineScopeEnum.AUTONOMOUS:
            return

        self.change_state(new_state)

    def handleCan(self, laps):
        """
        Subscriber callback for the lap counter. Does an internal state transition if required

        Args:
            laps: the UInt16 message containing the lap count
        """
        pass


node = AutonomousController()
