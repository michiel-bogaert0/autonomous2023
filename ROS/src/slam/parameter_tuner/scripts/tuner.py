import enum
from abc import ABC, abstractmethod
from itertools import product

import rospy
import yaml
from param import Param


class TunerModes(str, enum.Enum):
    NONE = "none"
    SUM = "sum"
    LIST = "list"
    MULTI = "multi"


def getTuner(config_file: str):
    with open(config_file, "r") as f:
        data = yaml.safe_load(f)
    mode = data["mode"]

    if mode == TunerModes.SUM:
        return SumTuner(data["param"], data[mode])
    elif mode == TunerModes.LIST:
        return ListTuner(data["param"], data[mode])
    elif mode == TunerModes.MULTI:
        return MultiTuner(data["param"], data[mode])
    return DefaultTuner(data["param"])


class Tuner(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def change(self):
        pass

    @abstractmethod
    def simulation_finished(self):
        pass


class SingleTuner(Tuner):
    def __init__(self, param_config) -> None:
        self.param = Param(param_config)

    def change(self):
        return self.param.set_parameter(self.change_tuner())

    @abstractmethod
    def change_tuner(self):
        return self.param.previous

    @abstractmethod
    def simulation_finished(self):
        pass


class MultiTuner(Tuner):
    def __init__(self, p, multi_config) -> None:
        Tuner.__init__(self)
        self.params = []
        self.values = []

        for param_config in multi_config:
            self.params.append(Param(param_config["param"]))
            self.values.append(param_config["values"])

        self.combinations = list(product(*self.values))
        rospy.loginfo(f"Combinations: {self.combinations}")
        self.counter = 0

    def change(self):
        for i, param in enumerate(self.params):
            param.set_parameter(self.combinations[self.counter][i])
        self.counter += 1

        # TODO: return all params (has to be implemented in main and data)
        return 0

    def simulation_finished(self):
        return self.counter >= len(self.combinations) - 1


class SumTuner(SingleTuner):
    def __init__(self, param_config, sum_config) -> None:
        SingleTuner.__init__(self, param_config)
        self.sum = sum_config["add_value"]
        self.simulation_count = sum_config["simulation_count"]
        self.counter = 0

    def change_tuner(self):
        self.counter += 1
        return self.param.previous + self.sum

    def simulation_finished(self):
        return self.counter >= self.simulation_count


class ListTuner(SingleTuner):
    def __init__(self, param_config, list_config) -> None:
        SingleTuner.__init__(self, param_config)
        self.values = list_config["values"]
        self.counter = 0

    def change_tuner(self):
        param = self.values[self.counter]
        self.counter += 1
        return param

    def simulation_finished(self):
        return self.counter >= len(self.values) - 1


class DefaultTuner(SingleTuner):
    def __init__(self, param_config) -> None:
        SingleTuner.__init__(self, param_config)

    def change_tuner(self):
        return self.param.previous

    def simulation_finished(self):
        return True
