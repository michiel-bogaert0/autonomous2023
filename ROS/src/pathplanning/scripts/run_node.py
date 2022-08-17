#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from fs_msgs.msg import Track as ROSTrack
from geometry_msgs.msg import Point, PoseArray, Pose, Quaternion
from node_fixture.node_fixture import AddSubscriber, ROSNode
from pathplanning.rrt import Rrt
from pathplanning.triangulator import Triangulator
from std_msgs.msg import Header
from ugr_msgs.msg import Observation, Observations
from tf_conversions import quaternion_from_euler


class PathPlanning(ROSNode):
    """Path planning node. Calculates and publishes path based on observations."""

    def __init__(self) -> None:
        """Initialize node"""
        super().__init__("exploration_mapping", False)

        MAP_TOPIC = "/pathplanning/local/map"
        OUTPUT_TOPIC = "/pathplanning/path"

        self.params = {}
        # Defines which algorithm to run triangulatie ("tri") or RRT ("RRT")
        self.params["algo"] = rospy.get_param("algoritme", "tri")
        # Load at least all params from config file via ros parameters
        # The distance by which the car drives every update
        self.params["expand_dist"] = rospy.get_param("expand_dist", 0.5)
        # The distance the car can see in front
        self.params["plan_dist"] = rospy.get_param("plan_dist", 12.0)
        # The amount of branches generated
        self.params["max_iter"] = rospy.get_param(
            "max_iter", 100 if self.params["expand_dist"] == "tri" else 750
        )

        # Early prune settings
        # The maximum angle (rad) for a branch to be valid (sharper turns will be pruned prematurely)
        self.params["max_angle_change"] = rospy.get_param("max_angle_change", 0.5)
        # The radius around a obstacle (cone) where no path can be planned
        # Should be at least the half the width of the car
        self.params["safety_dist"] = rospy.get_param("safety_dist", 1)

        # Extra parameters for RRT
        # Minimum or average width of the track
        # Used to estimate middle one side of cones is missing.
        self.params["track_width"] = rospy.get_param("track_width", 3)
        # Maximum width of the track used to detect if RRT node is possibly out of the track
        self.params["max_track_width"] = rospy.get_param("max_track_width", 4)
        # Used for RRT* variant to define radius to optimize new RRT node
        # When set to None, will be twice max_dist (expand_dist*3)
        self.params["search_rad"] = rospy.get_param("search_rad", None)
        # Iteration threshold which triggers parameter update. (3/4 of max_iter seems to be ok)
        self.params["iter_threshold"] = rospy.get_param("iter_threshold", 560)
        # Percentage to increase maximum angle when parameter update is triggered.
        self.params["angle_inc"] = rospy.get_param("angle_inc", 0.2)
        # Factor in to increase maximum angle to create more chance for edges.
        self.params["angle_fac"] = rospy.get_param("angle_fac", 1.5)

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
                self.params["track_width"],
                self.params["max_track_width"],
                self.params["search_rad"],
                self.params["iter_threshold"],
                self.params["angle_inc"],
                self.params["angle_fac"],
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

    def receive_new_map(self, _, track: Observations):
        """Receives observations from input topic.

        Args:
            _: Not used
            track: The observations/message on input topic.
        """
        self.cones = np.array(
            [
                [obs.location.x, obs.location.y, obs.observation_class]
                for obs in track.observations
            ]
        )

        # Compute
        self.compute(track.header)

    def compute(self, header: Header) -> None:
        """Calculate path and publish it.

        Args:
            header: Header of input message.
        """
        # t = time()
        path = self.algorithm.get_path(self.cones)
        # print('Algo: ', time()-t)

        # Calculate orientations
        yaws = np.arctan2(path[:, 1], path[:, 0])
        orientations = np.array(
            quaternion_from_euler(np.zeros_like(yaws), np.zeros_like(yaws), yaws)
        )

        poses: list(Pose) = []
        for idx in range(len(path)):
            pose: Pose = Pose()
            position: Point = Point()
            orientation: Quaternion = Quaternion()

            # Fill position from path point
            position.x = path[idx][0]
            position.y = path[idx][1]
            position.z = 0

            # Fill orientation
            orientation.x = orientations[0][idx]
            orientation.y = orientations[1][idx]
            orientation.z = orientations[2][idx]
            orientation.w = orientations[3][idx]

            # Fill pose and add to array
            pose.position = position
            pose.orientation = orientation
            poses += [pose]

        output: PoseArray = PoseArray()
        output.header.frame_id = header.frame_id
        output.poses = poses
        output.header.stamp = rospy.Time.now()

        pub = rospy.Publisher(PathPlanning.OUTPUT_TOPIC, PoseArray)
        pub.publish(output)


node = PathPlanning()
node.start()
