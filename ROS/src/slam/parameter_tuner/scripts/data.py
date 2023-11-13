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
    def __init__(self, nr: int, parameters) -> None:
        self.nr = nr
        self.parameters = parameters
        self.duration = 0
        self.stop_reason = SimulationStopEnum.NONE
        self.avgDistanceToConeSLAM = []

    def to_str(self):
        # parameters_str = ""
        # for param in self.parameters:
        #     (name, value) = param
        #     parameters_str += "\n\t\t" + name + ": " + str(value)
        return f"Simulation: {self.nr}\n\tParameters: {self.parameters}\n\tDuration (sec): {self.duration.to_sec()}\n\tStop reason: {self.stop_reason}\n\tEvaluation: {self.avgDistanceToConeSLAM}\n"
