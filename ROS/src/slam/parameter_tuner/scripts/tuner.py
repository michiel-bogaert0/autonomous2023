#! /usr/bin/python3
import rospy
from node_launcher.node_launcher import NodeLauncher

from time import sleep

class Tuner:
    def __init__(self) -> None:
        """
        Parameter tuner
        """
        rospy.init_node("parameter_tuner")

        self.launcher = NodeLauncher()

        while not rospy.is_shutdown():
            rospy.loginfo("node works")


            sleep(0.1)


node = Tuner()
