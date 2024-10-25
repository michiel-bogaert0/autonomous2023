#!/usr/bin/env python3

import numpy as np
import rospy
import tf2_ros as tf
from distance_path import DistancePath
from geometry_msgs.msg import Pose, PoseStamped
from local_boundaries import LocalBoundaries
from nav_msgs.msg import Path
from node_fixture.managed_node import ManagedNode
from std_msgs.msg import UInt16
from ugr_msgs.msg import (
    Boundaries,
    ObservationWithCovarianceArrayStamped,
    PathWithIds,
    PoseStampedWithIds,
)


class BoundaryEstimation(ManagedNode):
    def __init__(self):
        super().__init__("boundary_estimation")
        self.boundary_pub = super().AddPublisher(
            "/output/boundaries", Boundaries, queue_size=10
        )
        self.path_pub = super().AddPublisher("/output/path", PathWithIds, queue_size=10)
        self.path_pub2 = super().AddPublisher("/path2", PathWithIds, queue_size=10)
        self.left_boundary_pub = super().AddPublisher(
            "/output/debug/left_boundary", Path, queue_size=10
        )
        self.right_boundary_pub = super().AddPublisher(
            "/output/debug/right_boundary", Path, queue_size=10
        )
        self.lap_complete_sub = super().AddSubscriber(
            "/ugr/car/lapComplete", UInt16, self.lap_complete_callback
        )
        self.local = rospy.get_param("~local", False)
        self.strategy = rospy.get_param("~middle_line_strategy", "angle")
        self.original_map = None
        self.map = None
        self.base_link_frame = "ugr/car_base_link"
        self.slam_frame = "ugr/slam_base_link"
        self.world_frame = "ugr/map"
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.startpoint = None
        self.left_boundary = []
        self.right_boundary = []
        self.left_boundary_ids = []
        self.right_boundary_ids = []
        self.global_path_ids = []
        self.closed = False
        self.last_used_left = None
        self.last_used_right = None
        self.first_left = None
        self.first_right = None
        # self.global_path = []
        self.spin()

    def doActivate(self):
        self.map_sub = super().AddSubscriber(
            "/input/local_map",
            ObservationWithCovarianceArrayStamped,
            self.receive_new_map,
        )
        self.local_algorithm = LocalBoundaries()
        self.path_finder = DistancePath()

    def lap_complete_callback(self, msg):
        if msg.data == 1:
            self.closed = True

    def receive_new_map(self, map):
        if map is None:
            return
        cones = np.zeros((len(map.observations), 4))
        for i, obs in enumerate(map.observations):
            cones[i] = [
                obs.observation.location.x,
                obs.observation.location.y,
                obs.observation.observation_class,
                obs.observation.id,
            ]
        self.compute(cones, map.header)

    def compute(self, cones, header):
        (
            left_local,
            right_local,
            local_path_with_ids,
        ) = self.local_algorithm.get_boundaries(cones)
        if (
            left_local == []
            or right_local == []
            or left_local is None
            or right_local is None
        ):
            return

        local_path_with_ids = self.path_finder.get_path(left_local, right_local)

        # Bouwt een global boundary left & right
        # Waarom? voor wat wordt dit gebruikt?
        if left_local[1] not in self.left_boundary:
            if len(self.left_boundary_ids) == 0:
                self.first_left = left_local[1].id
                self.left_boundary_ids.append(left_local[1].id)
            else:
                if left_local[1].id not in self.left_boundary_ids:
                    self.left_boundary_ids.append(left_local[1].id)

        if right_local[1] not in self.right_boundary:
            if len(self.right_boundary_ids) == 0:
                self.first_right = right_local[1].id
                self.right_boundary_ids.append(right_local[1].id)
            else:
                if right_local[1].id not in self.right_boundary_ids:
                    self.right_boundary_ids.append(right_local[1].id)

        left_boundary = left_local
        right_boundary = right_local

        if self.closed:
            left_boundary = []
            right_boundary = []
            for id in self.left_boundary_ids:
                for cone in cones:
                    if cone[3] == id:
                        left_boundary.append(cone)
            for id in self.right_boundary_ids:
                for cone in cones:
                    if cone[3] == id:
                        right_boundary.append(cone)

        left_boundary_msg = Path()
        left_boundary_msg.header = header
        for cone in left_boundary:
            pose = PoseStamped()
            if self.closed:
                pose.pose.position.x = cone[0]
                pose.pose.position.y = cone[1]
            else:
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
            if self.closed:
                pose.pose.position.x = cone[0]
                pose.pose.position.y = cone[1]
            else:
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

        # Published het local path

        local_path_msg = PathWithIds()
        local_path_msg.header = header

        for point in local_path_with_ids:
            pose_with_ids = PoseStampedWithIds()
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            pose_with_ids.header = header
            pose_with_ids.pose = pose
            pose_with_ids.left_id = int(point[2])
            pose_with_ids.right_id = int(point[3])

            local_path_msg.poses.append(pose_with_ids)

        self.path_pub.publish(local_path_msg)


BoundaryEstimation()
