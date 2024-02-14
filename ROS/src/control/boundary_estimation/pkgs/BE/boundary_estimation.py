#!/usr/bin/env python3

import numpy as np
from color_connecter import get_color_lines
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture.managed_node import ManagedNode
from ugr_msgs.msg import Boundaries, ObservationWithCovarianceArrayStamped


class BoundaryEstimation(ManagedNode):
    def __init__(self):
        super().__init__("boundary_estimation")
        self.boundary_pub = super().AddPublisher(
            "/output/boundaries", Boundaries, queue_size=10
        )
        self.left_boundary_pub = super().AddPublisher(
            "/output/debug/left_boundary", Path, queue_size=10
        )
        self.right_boundary_pub = super().AddPublisher(
            "/output/debug/right_boundary", Path, queue_size=10
        )

        self.spin()

    def doActivate(self):
        self.map_sub = super().AddSubscriber(
            "/input/local_map",
            ObservationWithCovarianceArrayStamped,
            self.receive_new_map,
        )

    def receive_new_map(self, map):
        cones = np.zeros((len(map.observations), 3))
        for i, obs in enumerate(map.observations):
            cones[i] = [
                obs.observation.location.x,
                obs.observation.location.y,
                obs.observation.observation_class,
            ]

        self.compute(cones, map.header)

    def compute(self, cones, header):
        left_boundary, right_boundary = get_color_lines(cones, header)

        left_boundary_msg = Path()
        left_boundary_msg.header = header
        for cone in left_boundary:
            pose = PoseStamped()
            pose.pose.position.x = cone.x
            pose.pose.position.y = cone.y
            pose.pose.position.z = 0

            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            pose.header = header
            left_boundary_msg.poses.append(pose)

        right_boundary_msg = Path()
        right_boundary_msg.header = header
        for cone in right_boundary:
            pose = PoseStamped()
            pose.pose.position.x = cone.x
            pose.pose.position.y = cone.y
            pose.pose.position.z = 0

            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            pose.header = header
            right_boundary_msg.poses.append(pose)

        boundaries = Boundaries()
        boundaries.left_boundary = left_boundary_msg
        boundaries.right_boundary = right_boundary_msg

        self.boundary_pub.publish(boundaries)
        self.left_boundary_pub.publish(left_boundary_msg)
        self.right_boundary_pub.publish(right_boundary_msg)


BoundaryEstimation()
