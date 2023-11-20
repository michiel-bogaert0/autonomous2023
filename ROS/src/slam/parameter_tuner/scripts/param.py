import rospkg
import rospy
import yaml


class Param:
    def __init__(self, param) -> None:
        self.yaml_path = rospkg.RosPack().get_path(param["pkg"]) + param["file"]
        self.parameter_name = param["name"]
        self.index = param["index"]

        if self.index == "None":
            self.index = None

        with open(self.yaml_path, "r") as f:
            self.data = list(yaml.safe_load_all(f))

        self.start = self.get_parameter()
        self.previous = self.start

    # get the value from the parameter
    def get_parameter(self):
        if self.index is None:  # if no index is given its just a parameter
            return self.data[0][self.parameter_name]
        # if an index is given its a list
        return self.data[0][self.parameter_name][self.index]

    # Set the new value of the parameter in the yaml file
    def set_parameter(self, parameter):
        self.previous = self.get_parameter()
        if self.index is None:  # if no index is given its just a parameter
            self.data[0][self.parameter_name] = parameter
        else:
            self.data[0][self.parameter_name][self.index] = parameter

        with open(self.yaml_path, "w") as file:
            yaml.dump_all(self.data, file, sort_keys=False)

        rospy.loginfo(
            f"Change parameter({self.parameter_name}) to {parameter} in yaml file"
        )
        return (self.parameter_name, parameter)

    def reset(self):
        self.set_parameter(self.start)
