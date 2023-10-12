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

        self.filename = rospy.get_param('~filename', "circle_R15.yaml")

        self.simulationLauncher = NodeLauncher()
        self.carLauncher = NodeLauncher()
        
        self.launch_simulation(self.filename)

        while not rospy.is_shutdown():
            

            sleep(0.1)

        self.simulationLauncher.shutdown()


    def launch_simulation(self, filename: str):
        rospy.loginfo("Launch simulation")
        self.simulationLauncher.launch_node("ugr_launch", "launch/external/simulation.launch", [str('filename:=' + filename)])
        try:
            self.simulationLauncher.run()
        except Exception as e:
            rospy.logerr("[SLAM] Node launching %s", str(e))


node = Tuner()
