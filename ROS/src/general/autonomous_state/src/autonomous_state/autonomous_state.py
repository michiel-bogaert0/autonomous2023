#! /usr/bin/python3
import rospy
from enum import Enum
from std_msgs.msg import UInt16
from std_srvs.srv import Empty
from ugr_msgs.msg import State
from node_launcher import NodeLauncher
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import AutonomousMission, SLAMStatesEnum, AutonomousStatesEnum, StateMachineScopeEnum



class AutonmousController:
    def __init__(self) -> None:
        """
        Autonomous controller
        """
        rospy.init_node("as_controller")

        self.state = AutonomousStatesEnum.ASOFF

        self.launcher = NodeLauncher()

        rospy.Subscriber("/state", State, self.handle_state_change)
        rospy.Subscriber("/input/can", UInt16, self.handle_can)
        rospy.Subscriver("/input/odom", Odometry, self.handle_odom)

        self.state_publisher = rospy.Publisher("/state", State, queue_size=10)

        while not rospy.is_shutdown():
            
            rospy.sleep(0.01)

    def handle_odom(self, odom: Odometry):
        """
        Checks if we are currently in ASSTOP and if speed becomes almost zero, we do a state change to ASFINISHED

        Args:
            odom: the odometry message containing speed information
        """

        if self.state == AutonomousStatesEnum.ASSTOP:
            # Check if our speed is almost zero
            if odom.twist.twist.linear.x < 0.1:
                pass

    def handle_state_change(self, state: State):
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

            if self.state == AutonomousStatesEnum.ASDRIVE and state.cur_state == SLAMStatesEnum.FINISHED:
                new_state = AutonomousStatesEnum.ASSTOP
            

        elif state.scope == StateMachineScopeEnum.AUTONOMOUS:
            return

        self.state_publisher.publish(
            State(scope=StateMachineScopeEnum.AUTONOMOUS, prev_state=self.state, cur_state=new_state)
        )
        self.state = new_state

    def handleCan(self, laps):
        """
        Subscriber callback for the lap counter. Does an internal state transition if required

        Args:
            laps: the UInt16 message containing the lap count
        """


node = AutonmousController()
