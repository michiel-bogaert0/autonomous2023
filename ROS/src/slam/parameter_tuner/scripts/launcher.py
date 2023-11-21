import os
import signal
import subprocess
from abc import ABC, abstractmethod

import rospy
from node_launcher.node_launcher import NodeLauncher


def getLauncher(logging: bool):
    """
    Returns an instance of Launcher based on the logging parameter.
    """
    if logging:
        return LauncherLogging()
    else:
        return LauncherSubprocess()


class Launcher(ABC):
    """
    This class represents a launcher for simulation and car run in an autonomous system.
    """

    def __init__(self) -> None:
        self.simulation_running = False
        self.car_running = False

    @abstractmethod
    def launch_simulation(self, map_filename: str):
        """
        Launches the simulation launchfile using the provided map_filename parameter.

        Args:
            map_filename (str): The filename to be passed as a parameter to the simulation launch file.

        """
        rospy.loginfo("Launch simulation")

    @abstractmethod
    def launch_car(self, mission: str):
        """
        Launches the run launchfile using the provided mission parameter.

        Args:
            mission (str): The mission to be passed as a parameter to the run launch file.

        """
        rospy.loginfo("Launch car run")

    def shutdown(self):
        """
        Shuts down the system by calling the shutdown_car() and shutdown_simulation() methods.
        """
        self.shutdown_car()
        self.shutdown_simulation()

    @abstractmethod
    def shutdown_simulation(self):
        rospy.loginfo("Shutdown Simulation")

    @abstractmethod
    def shutdown_car(self):
        rospy.loginfo("Shutdown car run")


class LauncherLogging(Launcher):
    """A class that extends the Launcher class and provides logging functionality for launching simulation and car nodes."""

    def __init__(self) -> None:
        super().__init__()
        self.simulationLauncher = NodeLauncher()
        self.carLauncher = NodeLauncher()

    def launch_simulation(self, map_filename: str):
        """Launches the simulation node with the specified map filename.

        Args:
            map_filename (str): The filename of the map to be used for simulation.
        """
        super().launch_simulation(map_filename)
        self.simulationLauncher.launch_node(
            "ugr_launch",
            "launch/external/simulation.launch",
            [str("filename:=" + map_filename)],
        )
        try:
            self.simulationLauncher.run()
            self.simulation_running = True
        except Exception as e:
            rospy.logerr("Simulation launch: %s", str(e))
            self.simulation_running = False

    def launch_car(self, mission: str):
        """Launches the car node with the specified mission.

        Args:
            mission (str): The mission to be executed by the car.
        """
        super().launch_car(mission)
        self.carLauncher.launch_node(
            "ugr_launch", "launch/internal/run.launch", [str("mission:=" + mission)]
        )
        try:
            self.carLauncher.run()
            self.car_running = True
        except Exception as e:
            rospy.logerr("Car run launch: %s", str(e))
            self.car_running = False

    def shutdown_simulation(self):
        """Shuts down the simulation node."""
        if not self.simulation_running:
            return
        super().shutdown_simulation()
        self.simulationLauncher.shutdown()
        self.simulation_running = False

    def shutdown_car(self):
        """Shuts down the car node."""
        if not self.car_running:
            return
        super().shutdown_car()
        self.carLauncher.shutdown()
        self.car_running = False


class LauncherSubprocess(Launcher):
    """
    A class that extends the Launcher class and provides methods to launch and shutdown simulation and car processes.
    """

    def __init__(self) -> None:
        super().__init__()

    def launch_simulation(self, map_filename: str):
        """
        Launches the simulation process with the specified map filename.

        Args:
            map_filename (str): The filename of the map to be used in the simulation.
        """
        super().launch_simulation(map_filename)
        self.simulationProcess = subprocess.Popen(
            f"roslaunch ugr_launch simulation.launch filename:={map_filename} >/dev/null 2>&1",
            stdout=subprocess.PIPE,
            shell=True,
            preexec_fn=os.setsid,
        )
        self.simulation_running = True

    def launch_car(self, mission: str):
        """
        Launches the car process with the specified mission.

        Args:
            mission (str): The mission to be executed by the car.
        """
        super().launch_car(mission)
        self.carProcess = subprocess.Popen(
            f"roslaunch ugr_launch run.launch mission:={mission} >/dev/null 2>&1",
            stdout=subprocess.PIPE,
            shell=True,
            preexec_fn=os.setsid,
        )
        self.car_running = True

    def shutdown_simulation(self):
        """
        Shuts down the simulation process if it is running.
        """
        if not self.simulation_running:
            return
        super().shutdown_simulation()
        os.killpg(os.getpgid(self.simulationProcess.pid), signal.SIGTERM)
        self.simulation_running = False

    def shutdown_car(self):
        """
        Shuts down the car process if it is running.
        """
        if not self.car_running:
            return
        super().shutdown_car()
        os.killpg(os.getpgid(self.carProcess.pid), signal.SIGTERM)
        self.car_running = False
