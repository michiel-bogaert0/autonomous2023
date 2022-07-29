#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import logging
from logging.config import dictConfig

import numpy as np
import rospy
from fs_msgs.msg import Track as ROSTrack
from node_fixture.node_fixture import AddSubscriber, ROSNode

from pathplanning.rrt import Rrt
from pathplanning.triangulator import Triangulator

logging_config = dict(
    version=1,
    formatters={
        "with_time": {"format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s"},
        "without_time": {"format": "%(name)s - %(levelname)s - %(message)s"},
    },
    handlers={
        "debug_handler": {
            "class": "logging.StreamHandler",
            "formatter": "without_time",
            "level": logging.DEBUG,
        },
        "warning_handler": {
            "class": "logging.FileHandler",
            "filename": "pathfinding.log",
            "formatter": "with_time",
            "level": logging.WARNING,
        },
    },
    root={
        "handlers": ["debug_handler", "warning_handler"],
        "level": logging.DEBUG,
    },
)

dictConfig(logging_config)


class PathPlanning(ROSNode):
    def __init__(self) -> None:
        super().__init__("exploration_mapping", False)

        MAP_TOPIC = "/pathfinding/local/map"

        self.params = {}
        self.params["algo"] = rospy.get_param("expand_dist", "tri")
        # Load at least all params from config file via ros parameters
        # The distance by which the car drives every update
        self.params["expand_dist"] = rospy.get_param("expand_dist", 0.5)
        # The distance the car can see in front
        self.params["plan_dist"] = rospy.get_param("plan_dist", 12.0)
        # The amount of branches generated
        self.params["max_iter"] = rospy.get_param("max_iter", 100)

        # Early prune settings
        # The maximum angle (rad) for a branch to be valid (sharper turns will be pruned prematurely)
        self.params["max_angle_change"] = rospy.get_param("max_angle_change", 0.5)
        # The radius around a obstacle (cone) where no path can be planned
        # Should be at least the half the width of the car
        self.params["safety_dist"] = rospy.get_param("safety_dist", 1)

        # TODO: load more params

        # Initialize cones, might not be needed
        track_layout = rospy.wait_for_message(MAP_TOPIC, ROSTrack)
        self.cones = np.array(
            [
                [cone.location.x, cone.location.y, cone.color]
                for cone in track_layout.track
            ]
        )

        if self.params["algo"] == "rrt":
            self.algorithm = Rrt(
                self.params["expand_dist"] * 3,
                self.params["plan_dist"],
                self.params["max_iter"],
                self.params["max_angle_change"],
                self.params["safety_dist"],
            )
        else:
            self.algorithm = Triangulator(
                self.params["max_iter"],
                self.params["plan_dist"],
                self.params["max_angle_change"],
                self.params["safety_dist"],
            )

        AddSubscriber(MAP_TOPIC, 1)(self.receive_new_map)
        self.add_subscribers()

    def receive_new_map(self, _, track: ROSTrack):
        self.cones = np.array(
            [[cone.location.x, cone.location.y, cone.color] for cone in track.track]
        )

        # Compute
        self.compute()

    def compute(self):
        try:
            # t = time()
            path, edge_centers, root = self.algorithm.get_path(self.cones)
            # print('Algo: ', time()-t)
        except:
            pass

        # TODO: publish to some topic


node = PathPlanning()
node.start()
