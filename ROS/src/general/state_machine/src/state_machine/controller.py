#!/usr/bin/env python3
import rospy
from enum import Enum
from std_msgs.msg import Empty

class State(Enum):
    Racing = 1
    Exploring = 2
    Idle = 3

class StateMachine:
    def __init__(self) -> None:
        """
        Starts the node and keeps it running
        """
        rospy.init_node("state_machine")
        self.publisher = rospy.Publisher('StartState', State)
        self.isfinishednode = rospy.Service('finished_init',Empty )
        rospy.spin()
    
    def ChangeState(self,nextState: State) -> bool:
        """
        Checks if currentState is the same as the nextState, and switches to next state
        Args:
            The nextState
        Return:
            returns if the it hasn't failed to switch to the next state
        """
        if (self.currentState == nextState):
            return False
        self.publisherEndState.publish(self.currentState)


        # if you want to add the posibility so the initialization of the nodes. 
        # we can do this but they need to send a confirmation that the initialization is finished
        self.currentState = nextState
        self.publisherEndState.get_num_connections()