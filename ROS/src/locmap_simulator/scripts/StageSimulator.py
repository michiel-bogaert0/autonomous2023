#!/usr/bin/env python3
import rospy
from node_fixture.node_fixture import ROSNode


class StageSimulator(ROSNode):
    def __init__(self, name: str) -> None:
        """
        Requires a name to be used to namespace stuff
        """
        super().__init__("stagesimulator_" + name)

        self.name = name
        self.publish_rate = rospy.get_param("~publish_rate", 10)
        self.timer = rospy.Timer(
            rospy.Duration(1 / self.publish_rate),
            self.simulate,
        )

    def get_namespace(self, path: str) -> str:
        """
        Builds a namespace based on the given simulation name script and the required subpath
        """
        return f"/stage_simulation/{self.name}/{path.lstrip('/')}"

    def simulate(self, timer):
        """
        Dummy function that gets called by rospy.Timer
        """
