#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from node_fixture.managed_node import ManagedNode
from std_msgs.msg import UInt16
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped, PathWithIds


class PathBuilder(ManagedNode):
    def __init__(self):
        super().__init__("path_builder")
        self.spin()

    def doConfigure(self):
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.path_publisher = super().AddPublisher("/output/path", Path, queue_size=10)
        self.lap_complete_sub = super().AddSubscriber(
            "/ugr/car/lapComplete", UInt16, self.lap_complete_callback
        )
        self.explo_done = False
        self.global_path_ids = []

        self.cones = None
        self.merges = {}

        self.vis_pub = super().AddPublisher(
            "/output/closest_point", PointStamped, queue_size=10  # warning otherwise
        )

    def doActivate(self):
        self.map_sub = super().AddSubscriber(
            "/input/local_map",
            ObservationWithCovarianceArrayStamped,
            self.receive_new_map,
        )
        self.path_sub = super().AddSubscriber(
            "/input/path", PathWithIds, self.path_callback
        )

        self.global_path_enabled = rospy.get_param("~enable_global_path", False)

    def lap_complete_callback(self, msg: UInt16):
        if msg.data >= 1:
            self.explo_done = True

    def receive_new_map(self, map):
        if map is None:
            return
        self.cones = np.zeros((len(map.observations), 4))
        self.merges = {}
        for i, obs in enumerate(map.observations):
            self.cones[i] = [
                obs.observation.location.x,
                obs.observation.location.y,
                obs.observation.observation_class,
                obs.observation.id,
            ]
            self.merges[obs.observation.id] = obs.observation.merged_ids

    def path_callback(self, msg: PathWithIds):
        """
        Creates global path, given pathplanning path and IDs
        """
        # Store IDs of first pose of each pathplanning path
        closest_point_ids = (msg.poses[0].left_id, msg.poses[0].right_id)

        # Visualize center between closest points
        for cone in self.cones:
            if (
                cone[3] == closest_point_ids[0]
                or closest_point_ids[0] in self.merges[cone[3]]
            ):
                left_cone = cone
            if (
                cone[3] == closest_point_ids[1]
                or closest_point_ids[1] in self.merges[cone[3]]
            ):
                right_cone = cone

        if left_cone is not None and right_cone is not None:
            new_point = (
                (left_cone[0] + right_cone[0]) / 2,
                (left_cone[1] + right_cone[1]) / 2,
            )
        else:
            return

        # Publish target point for visualization
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.base_link_frame
        point.point.x = new_point[0]
        point.point.y = new_point[1]

        self.vis_pub.publish(point)

        if closest_point_ids not in self.global_path_ids:
            self.global_path_ids.append(closest_point_ids)

        if not self.explo_done or not self.global_path_enabled:
            # Just publish the pathplanning path
            path = Path()
            path.header = msg.header
            for pose in msg.poses:
                path.poses.append(pose)

            self.path_publisher.publish(path)
        # Build global path based on saved IDs
        else:
            global_path = []
            for tuple in self.global_path_ids:
                left_cone = None
                right_cone = None
                for cone in self.cones:
                    if cone[3] == tuple[0]:
                        left_cone = cone
                    if cone[3] == tuple[1]:
                        right_cone = cone
                if left_cone is not None and right_cone is not None:
                    new_point = (
                        (left_cone[0] + right_cone[0]) / 2,
                        (left_cone[1] + right_cone[1]) / 2,
                    )
                    if len(global_path) >= 2:
                        dist_new_to_second_last_point = (
                            new_point[0] - global_path[-2][0]
                        ) ** 2 + (new_point[1] - global_path[-2][1]) ** 2
                        dist_last_to_second_last_point = (
                            global_path[-1][0] - global_path[-2][0]
                        ) ** 2 + (global_path[-1][1] - global_path[-2][1]) ** 2

                        if (
                            dist_new_to_second_last_point
                            > dist_last_to_second_last_point
                        ):
                            global_path.append(new_point)
                    else:
                        global_path.append(new_point)
            path = Path()
            path.header = msg.header
            for point in global_path:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                path.poses.append(pose)
            self.path_publisher.publish(path)


PathBuilder()
