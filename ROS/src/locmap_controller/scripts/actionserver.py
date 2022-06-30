#! /usr/bin/env python3

#! WIP

import rospy
import actionlib
import roslaunch

from locmap_controller.msg import (
    StartSimulationActionGoal,
    StartSimulationAction,
    StartSimulationActionFeedback,
    StartSimulationActionResult,
    StartSimulationGoal,
)


class LocMapActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "start_locmap_simulation", StartSimulationAction, self.start_locmap_simulation, False
        )
        self.server.start()

    def start_locmap_simulation(self, goal: StartSimulationActionGoal):
        pass


if __name__ == "__main__":
    rospy.init_node("locmap_action_server")
    server = LocMapActionServer()
    rospy.spin()
