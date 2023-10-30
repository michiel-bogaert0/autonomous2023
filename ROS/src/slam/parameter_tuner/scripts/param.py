import enum

import rospy
import yaml


class TunerModes(str, enum.Enum):
    NONE = "none"
    SUM = "sum"


def getTuner(mode: TunerModes):
    if mode == TunerModes.SUM:
        return SumParam
    else:
        return Param


class Param:
    def __init__(self, yaml_path: str, parameter_name: str) -> None:
        self.yaml_path = yaml_path
        self.parameter_name = parameter_name

        with open(self.yaml_path, "r") as f:
            self.data = list(yaml.safe_load_all(f))

        self.previous = self.get_parameter()

    def change(self):
        self.set_parameter(self.change_tuner(self.previous))
        return self.previous

    def change_tuner(self, p):
        return p

    def get_parameter(self):
        return self.data[0][self.parameter_name]

    def set_parameter(self, parameter):
        self.data[0][self.parameter_name] = parameter
        with open(self.yaml_path, "w") as file:
            yaml.dump_all(self.data, file, sort_keys=False)

        rospy.loginfo(
            f"Change parameter({self.parameter_name}) to {parameter} in yaml file"
        )


class SumParam(Param):
    def __init__(self, yaml_path: str, parameter_name: str, value) -> None:
        Param.__init__(self, yaml_path, parameter_name)
        self.value = value

    def change_tuner(self, p):
        return p - self.value
