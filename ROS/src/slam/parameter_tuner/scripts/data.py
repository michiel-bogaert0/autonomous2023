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
    def __init__(self) -> None:
        pass

    @abstractmethod
    def add(self, simulation_data):
        pass

    @abstractmethod
    def end(self):
        pass


class PrintData(Data):
    def __init__(self) -> None:
        self.data = []

    def add(self, simulation_data):
        self.data.append(simulation_data)

    def end(self):
        str = ""
        for simulation_data in self.data:
            str += simulation_data.to_str()
        rospy.loginfo(f"Data:\n{str}")


class CsvData:
    def __init__(self) -> None:
        folder_path = rospkg.RosPack().get_path("parameter_tuner") + "/tuner_data"
        if not os.path.exists(folder_path):
            # Create the new folder
            os.makedirs(folder_path)
            print(f"Folder '{folder_path}' created")

        current_time = datetime.datetime.now()
        filename = f"{current_time.strftime('%d-%m-%Y_%H:%M:%S')}.csv"

        self.path = folder_path + "/" + filename

    def add(self, simulation_data):
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
        pass


class SimulationData:
    def __init__(self, nr: int, parameters) -> None:
        self.nr = nr
        self.parameters = parameters
        self.duration = 0
        self.stop_reason = SimulationStopEnum.NONE
        self.avgDistanceToConeSLAM = []
        self.labelsConesSlam = []

    def to_str(self):
        # parameters_str = ""
        # for param in self.parameters:
        #     (name, value) = param
        #     parameters_str += "\n\t\t" + name + ": " + str(value)
        return f"Simulation: {self.nr}\n\tParameters: {self.parameters}\n\tDuration (sec): {self.duration.to_sec()}\n\tStop reason: {self.stop_reason}\n"

    def to_dict(self):
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
