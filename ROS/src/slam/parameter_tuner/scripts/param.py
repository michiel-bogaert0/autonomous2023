import rospy
import yaml


class Param:
    def __init__(self, yaml_path: str, parameter_name: str) -> None:
        self.yaml_path = yaml_path
        self.parameter_name = parameter_name

        with open(self.yaml_path, "r") as f:
            self.data = list(yaml.safe_load_all(f))

        self.previous = self.get_parameter()

    def change(self):
        self.previous -= 100
        self.set_parameter(self.previous)
        return self.previous

    def get_parameter(self):
        return self.data[0][self.parameter_name]

    def set_parameter(self, parameter):
        self.data[0][self.parameter_name] = parameter
        with open(self.yaml_path, "w") as file:
            yaml.dump_all(self.data, file, sort_keys=False)

        rospy.loginfo(
            f"Change parameter({self.parameter_name}) to {parameter} in yaml file"
        )
