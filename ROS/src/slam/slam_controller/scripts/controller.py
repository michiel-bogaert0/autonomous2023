#! /usr/bin/python3
import rospy
from enum import Enum
from std_msgs.msg import UInt16MultiArray, UInt16
from std_srvs.srv import Empty
"""
this is temp to init states, this will be changed in the future
"""
class State(Enum):
    Racing = 1
    Exploring = 2
    Idle = 3

class Controller:
    resetLoopCloser = rospy.Subscriber
    totalAmountOfLaps = 0

    def DeployRacingMode():
        """
        Initialize the racing mode and the packages around it
        """
        rospy.loginfo("Deploy the racing mode")
    
    def DestroyRacingMode():
        """
        Destroy the package is RacingMode
        """
        rospy.loginfo("Destroy the racing mode")

    def DeployExploringMode():
        """
        Initialize the racing mode and the packages around it
        """
        rospy.loginfo("Deploy the exploring mode")

    def DestoryExploringMode():
        """
        Destroy the package is explorationMode
        """
        rospy.loginfo("Destroy the exploring mode")

    def FromExploringToDriving(self):
        """
        Setup the packages for deletion and creation when changing from explorating to driving
        """
        self.DestoryExploringMode()
        self.DeployRacingMode()
    
    def FromDrivingToIdle(self):
        """
        Setup the packages for creation and deletion when changing to Idle from driving
        """
        self.DestroyRacingMode()
        self.totalAmountOfLaps = 0
        #is possible to delete even more
    
    def FromExplorationToIdle(self):
        """
        Setup the packages for creation and deletion when changing to Idle from Exploration
        """
        self.totalAmountOfLaps = 0
        self.DestoryExploringMode()
    
    def FromIdleToExploring(self):
        """
        Setup the packages for creation and deletion when changing to Idle from Exploration
        """
        self.DeployExploringMode()
        
    def lapFinished(self, laps):
        """
        this only needs to happen if the currentState is in exploration
        """
        
        self.totalAmountOfLaps +=1
        if(self.totalAmountOfLaps == 1):
            rospy.ServiceProxy('/reset_closure', Empty)
    
    
    def StartChangingState(self,data):
        """
        starts up the nodes that is used in the different stages
        and destroys the current nodes

        Args:
            the next state that the upper controller passes through
        """
        previousState = State(data[0])
        nextState = State(data[1])
        if(previousState == State.Exploring):
            if(nextState == State.Racing):
                self.FromExploringToDriving()
            elif(nextState == State.Idle):
                self.FromExplorationToIdle()
        elif (previousState == State.Racing):
            if(nextState == State.Idle):
                self.FromDrivingToIdle()
        elif (previousState == State.Idle):
            if(nextState == State.Exploring):
                self.FromIdleToExploring()
        else: 
            #other states
            print("Other state")

    def __init__(self, service_class) -> None:
        rospy.init_node("controller")
        rospy.logerr("information")
        #the array represents the current state at 0 and the next state at 1 but will be changed in the future
        rospy.Subscriber("stateChange",UInt16MultiArray , self.StartChangingState)
        self.resetLoopCloser =  rospy.Subscriber("/output/loopClosure", UInt16, self.lapFinished)
        rospy.spin()

node = Controller(None)