import enum


class SimulationStopEnum(str, enum.Enum):
    NONE = "none"
    TimeLimit = "timelimit"
    ASFINISHED = "asfinished"


class Data:
    def __init__(self) -> None:
        self.data = []

    def add(self, simulation_data):
        self.data.append(simulation_data)

    def to_str(self):
        str = ""
        for simulation_data in self.data:
            str += simulation_data.to_str()
        return str


class SimulationData:
    def __init__(self, nr: int, parameter) -> None:
        self.nr = nr
        self.parameter = parameter
        self.duration = 0
        self.stop_reason = SimulationStopEnum.NONE

    def to_str(self):
        return f"Simulation: {self.nr}\n\tParameter: {self.parameter}\n\tDuration (sec): {self.duration.to_sec()}\n\tStop reason: {self.stop_reason}\n"
