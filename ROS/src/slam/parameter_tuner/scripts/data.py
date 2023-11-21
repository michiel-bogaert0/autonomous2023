import csv
import datetime
import enum
import os
from abc import ABC, abstractmethod

import rospkg
import rospy


class SimulationStopEnum(str, enum.Enum):
    NONE = "none"
    TimeLimit = "timelimit"
    ASFINISHED = "asfinished"

    def __str__(self):
        return str(self.value)


def getDataHandler(method: str):
    if method == "print":
        return PrintData()
    if method == "csv":
        return CsvData()
    return PrintData()


class Data(ABC):
    """
    Abstract class to handle data.
    """

    def __init__(self) -> None:
        pass

    @abstractmethod
    def add(self, simulation_data):
        """
        Adds simulation data to the data handler.
        """
        pass

    @abstractmethod
    def end(self):
        """
        This method can be used for any cleanup or finalization tasks.
        """
        pass


class PrintData(Data):
    """
    Class to handle data.
    Just prints the data to the console at the end.
    """

    def __init__(self) -> None:
        self.data = []

    def add(self, simulation_data):
        self.data.append(simulation_data)

    def end(self):
        """
        Prints the data to the console.
        """
        str = ""
        for simulation_data in self.data:
            str += simulation_data.to_str()
        rospy.loginfo(f"Data:\n{str}")


class CsvData:
    """
    Class to handle CSV data storage and manipulation.
    """

    def __init__(self) -> None:
        """
        Initializes the CsvData object.

        Creates a folder to store the CSV files if it doesn't exist.
        Generates a unique filename based on the current time.
        """
        folder_path = rospkg.RosPack().get_path("parameter_tuner") + "/tuner_data"
        if not os.path.exists(folder_path):
            # Create the new folder
            os.makedirs(folder_path)
            print(f"Folder '{folder_path}' created")

        current_time = datetime.datetime.now()
        filename = f"{current_time.strftime('%d-%m-%Y_%H:%M:%S')}.csv"

        self.path = folder_path + "/" + filename

    def add(self, simulation_data):
        """
        Adds simulation data to the CSV file.

        Args:
            simulation_data: The simulation data to be added.

        """
        data = simulation_data.to_dict()
        file_exists = False
        try:
            with open(self.path, "r"):
                file_exists = True
        except FileNotFoundError:
            pass

        with open(self.path, "a", newline="") as csvfile:
            fieldnames = list(data.keys())
            csv_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            # If the file doesn't exist, write the headers
            if not file_exists:
                csv_writer.writeheader()

            # Append the new data to the CSV file
            csv_writer.writerow(data)

    def end(self):
        """
        Placeholder method.

        This method can be used for any cleanup or finalization tasks.
        """
        pass


class SimulationData:
    """
    Class representing simulation data.

    Attributes:
        nr (int): The simulation number.
        parameters (list): The parameters used in the simulation.
        duration (float): The duration of the simulation in seconds.
        stop_reason (SimulationStopEnum): The reason for stopping the simulation.
        avgDistanceToConeSLAM (list): The average distance to cones in SLAM.
        labelsConesSlam (list): The labels of cones in SLAM.
    """

    def __init__(self, nr: int, parameters) -> None:
        self.nr = nr
        self.parameters = parameters
        self.duration = 0
        self.stop_reason = SimulationStopEnum.NONE
        self.avgDistanceToConeSLAM = []
        self.labelsConesSlam = []

    def to_str(self):
        """
        Convert the simulation data to a string representation.

        Returns:
            str: The string representation of the simulation data.
        """
        return f"Simulation: {self.nr}\n\tParameters: {self.parameters}\n\tDuration (sec): {self.duration.to_sec()}\n\tStop reason: {self.stop_reason}\n"

    def to_dict(self):
        """
        Convert the simulation data to a dictionary representation.

        Returns:
            dict: The dictionary representation of the simulation data.
        """
        params = []
        for param, value in self.parameters:
            params.append({param: value})
        return {
            "Simulation": self.nr,
            "Parameters": params,
            "Duration": self.duration.to_sec(),
            "StopReason": str(self.stop_reason),
            "avgDistanceToConeSLAM": self.avgDistanceToConeSLAM,
            "labelsConesSlam": self.labelsConesSlam,
        }
