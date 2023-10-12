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


    #
    def launch_simulation(self, filename: str):
        """
        Launches the simulation launchfile using the provided filename parameter.

        Args:
            filename (str): The filename to be passed as a parameter to the simulation launch file.

        """
        rospy.loginfo("Launch simulation")
        self.simulationLauncher.launch_node("ugr_launch", "launch/external/simulation.launch", [str('filename:=' + filename)])
        try:
            self.simulationLauncher.run()
        except Exception as e:
            rospy.logerr("Simulation launch: %s", str(e))


node = Tuner()
