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

        self.map_filename = rospy.get_param('~map', "circle_R15") + ".yaml"
        self.mission = rospy.get_param('~mission', "trackdrive")

        self.simulationLauncher = NodeLauncher()
        self.carLauncher = NodeLauncher()
        
        self.launch_simulation(self.map_filename)

        sleep(4)

        self.launch_car(self.mission)

        while not rospy.is_shutdown():
            

            sleep(0.1)

        self.carLauncher.shutdown()
        self.simulationLauncher.shutdown()

    #
    def launch_simulation(self, map_filename: str):
        """
        Launches the simulation launchfile using the provided map_filename parameter.

        Args:
            map_filename (str): The filename to be passed as a parameter to the simulation launch file.

        """
        rospy.loginfo("Launch simulation")
        self.simulationLauncher.launch_node("ugr_launch", "launch/external/simulation.launch", [str('filename:=' + map_filename)])
        try:
            self.simulationLauncher.run()
        except Exception as e:
            rospy.logerr("Simulation launch: %s", str(e))

    def launch_car(self, mission: str):
        """
        Launches the run launchfile using the provided mission parameter.

        Args:
            mission (str): The mission to be passed as a parameter to the run launch file.

        """
        rospy.loginfo("Launch car run")
        self.carLauncher.launch_node("ugr_launch", "launch/internal/run.launch", [str('mission:=' + mission)])
        try:
            self.carLauncher.run()
        except Exception as e:
            rospy.logerr("Car run launch: %s", str(e))



node = Tuner()
