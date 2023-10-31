import enum

import rospy
import yaml


class TunerModes(str, enum.Enum):
    NONE = "none"
    SUM = "sum"
    LIST = "list"


def getTuner(mode: TunerModes):
    if mode == TunerModes.SUM:
        return SumParam
    elif mode == TunerModes.LIST:
        return ListParam
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
        self.previous = self.set_parameter(self.change_tuner(self.previous))
        return self.previous

    # change the value of the parameter for the next simulation run
    def change_tuner(self, p):
        return p

    # get the value from the parameter
    def get_parameter(self):
        return self.data[0][self.parameter_name]

    # Set the new value of the parameter in the yaml file
    def set_parameter(self, parameter):
        self.data[0][self.parameter_name] = parameter
        with open(self.yaml_path, "w") as file:
            yaml.dump_all(self.data, file, sort_keys=False)

        rospy.loginfo(
            f"Change parameter({self.parameter_name}) to {parameter} in yaml file"
        )
        return parameter


class SumParam(Param):
    def __init__(self, yaml_path: str, parameter_name: str) -> None:
        Param.__init__(self, yaml_path, parameter_name)
        self.value = rospy.get_param("~sum_add_value", 0)

    def change_tuner(self, p):
        return p + self.value


class ListParam(Param):
    def __init__(self, yaml_path: str, parameter_name: str) -> None:
        Param.__init__(self, yaml_path, parameter_name)
        self.list = rospy.get_param("~list", [])
        self.index = 0

    def change_tuner(self, p):
        param = self.list[self.index]
        self.index += 1
        return param

    def get_number_of_simulations(self):
        return len(self.list)
