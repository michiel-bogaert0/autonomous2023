#! /usr/bin/python3
import rospy
from ugr_msgs.msg import State

from launcher import Launcher

from time import sleep

class Tuner:
    def __init__(self) -> None:
        """
        Parameter tuner
        """
        rospy.init_node("parameter_tuner")

        self.map_filename = rospy.get_param('~map', "circle_R15") + ".yaml"
        self.mission = rospy.get_param('~mission', "trackdrive")


        self.launcher = Launcher(False)
        
        rospy.Subscriber("/state", State, self.state_callback)
        

        self.launcher.launch_simulation(self.map_filename)
        sleep(1)
        self.launcher.launch_car(self.mission)

        while not rospy.is_shutdown():
            
            sleep(0.1)

        self.launcher.shutdown_car()
        self.launcher.shutdown_simulation()

    def state_callback(self, data: State):
        rospy.loginfo("Scope: %s, State: %s", data.scope, data.cur_state)
        return

node = Tuner()
