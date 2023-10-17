import rospy

from node_launcher.node_launcher import NodeLauncher

import subprocess
import signal
import os

class Launcher:
    def __init__(self, logging: bool = True) -> None:
        self.logging = logging
        self.simulation_running = False
        self.car_running = False
        if(self.logging):
            self.simulationLauncher = NodeLauncher()
            self.carLauncher = NodeLauncher()

    def launch_simulation(self, map_filename: str):
        """
        Launches the simulation launchfile using the provided map_filename parameter.

        Args:
            map_filename (str): The filename to be passed as a parameter to the simulation launch file.

        """
        rospy.loginfo("Launch simulation")
        self.simulation_running = True

        # run a parallel subprocess for launching if you don't want to log
        if(not self.logging):
            self.simulationProcess = subprocess.Popen(f"roslaunch ugr_launch simulation.launch filename:={map_filename} >/dev/null 2>&1", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            return

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
        self.car_running = True

        # run a parallel subprocess for launching if you don't want to log
        if(not self.logging):
            self.carProcess = subprocess.Popen(f"roslaunch ugr_launch run.launch mission:={mission} >/dev/null 2>&1", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            return

        self.carLauncher.launch_node("ugr_launch", "launch/internal/run.launch", [str('mission:=' + mission)])
        try:
            self.carLauncher.run()
        except Exception as e:
            rospy.logerr("Car run launch: %s", str(e))
    

    def shutdown_simulation(self):
        if(not self.simulation_running):
            return
        rospy.loginfo("Shutdown Simulation")
        self.simulation_running = False
        if(self.logging):
            self.simulationLauncher.shutdown()
        else:
            # self.simulationProcess.terminate() # Doesn't work because of shell=True
            os.killpg(os.getpgid(self.simulationProcess.pid), signal.SIGTERM)

    def shutdown_car(self):
        if(not self.car_running):
            return
        rospy.loginfo("Shutdown car run")
        self.car_running = False
        if(self.logging):
            self.carLauncher.shutdown()
        else:
            # self.carProcess.kill()
            os.killpg(os.getpgid(self.carProcess.pid), signal.SIGTERM)