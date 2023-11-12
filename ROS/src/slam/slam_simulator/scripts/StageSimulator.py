#!/usr/bin/env python3
from abc import ABC, abstractmethod

import rospy
from node_fixture.fixture import (
    DiagnosticArray,
    DiagnosticStatus,
    ROSNode,
    create_diagnostic_message,
)


class StageSimulator(ROSNode, ABC):
    def __init__(self, name: str) -> None:
        """
        This is a generic class to simulate a 'stage'

        Args:
            name: Requires a name to be used to namespace stuff
        """
        super().__init__("stagesimulator_" + name)

        self.name = name
        self.publish_rate = rospy.get_param("~publish_rate", 10)
        self.timer = rospy.Timer(
            rospy.Duration(1 / self.publish_rate),
            self.simulate,
        )

        # Diagnostics Publisher
        self.diagnostics = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[SLAM SIM] Stage Simulator Status",
                message="Started.",
            )
        )

    def get_namespace(self, path: str) -> str:
        """
        Builds a namespace based on the given simulation name script and the required subpath
        """
        return f"/stage_simulation/{self.name}/{path.lstrip('/')}"

    @abstractmethod
    def simulate(self, timer):
        """
        Dummy function that gets called by rospy.Timer
        """
