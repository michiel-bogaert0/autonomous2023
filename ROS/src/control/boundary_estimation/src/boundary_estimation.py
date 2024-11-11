#!/usr/bin/env python3

import numpy as np
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
        self.left_boundary_pub = super().AddPublisher(
            "/output/debug/left_boundary", Path, queue_size=10
        )
        self.right_boundary_pub = super().AddPublisher(
            "/output/debug/right_boundary", Path, queue_size=10
        )
        self.lap_complete_sub = super().AddSubscriber(
            "/ugr/car/lapComplete", UInt16, self.lap_complete_callback
        )
        self.map = None
        self.left_boundary = []
        self.right_boundary = []
        self.left_boundary_ids = []
        self.right_boundary_ids = []
        self.closed = False
        self.spin()

    def doActivate(self):
        self.map_sub = super().AddSubscriber(
            "/input/local_map",
            ObservationWithCovarianceArrayStamped,
            self.receive_new_map,
        )
        self.local_boundaries_algorithm = LocalBoundaries()
        self.path_finder = DistancePath()

    def lap_complete_callback(self, msg):
        """
        Callback function to handle lap completion signal.
        Sets the `closed` attribute to True when the lap is complete.
        We can then find the global boundaries.
        """
        if msg.data == 1:
            self.closed = True

    def receive_new_map(self, map):
        """
        Receives a new map of observed cones. Extracts cone data from the
        message and passes it to the `compute` function.

        Parameters:
        - map: ObservationWithCovarianceArrayStamped containing the cones in the map.
        """
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
        """
        Calculates boundaries and path based on cone data. Publishes boundaries
        and path messages. Uses boundary IDs to retrieve boundaries once the lap is closed.

        Parameters:
        - cones: numpy array of cone data (x, y, class, ID).
        - header: header from the map.
        """
        left_boundary, right_boundary = self.local_boundaries_algorithm.get_boundaries(
            cones
        )
        if not left_boundary or not right_boundary:
            return

        local_path_with_ids = self.path_finder.get_path(left_boundary, right_boundary)

        # Update boundary IDs if they are not already in the list
        self.update_boundary_ids(
            left_boundary, self.left_boundary, self.left_boundary_ids
        )
        self.update_boundary_ids(
            right_boundary, self.right_boundary, self.right_boundary_ids
        )

        # When the lap is complete, we can find the global boundaries
        if self.closed:
            left_boundary, right_boundary = self.get_global_boundaries(cones)

        # Create and publish boundary messages
        left_boundary_msg = self.create_boundary_msg(left_boundary, header)
        right_boundary_msg = self.create_boundary_msg(right_boundary, header)

        boundaries = Boundaries()
        boundaries.left_boundary = left_boundary_msg
        boundaries.right_boundary = right_boundary_msg
        self.publish_boundaries(boundaries, left_boundary_msg, right_boundary_msg)

        # Create and publish path message
        local_path_msg = self.create_path_msg(local_path_with_ids, header)
        self.path_pub.publish(local_path_msg)

    def update_boundary_ids(self, boundary, boundary_list, boundary_ids_list):
        """
        Adds unique boundary IDs to the boundary ID list if not already present.

        Parameters:
        - boundary: current boundary to check.
        - boundary_list: list storing boundary points.
        - boundary_ids_list: list storing unique boundary IDs.
        """
        if boundary[1] not in boundary_list and (
            len(boundary_ids_list) == 0 or boundary[1].id not in boundary_ids_list
        ):
            boundary_ids_list.append(boundary[1].id)

    def get_global_boundaries(self, cones):
        """
        Retrieves global boundaries by matching cone IDs with previously stored boundary IDs
        when the lap is closed.

        Parameters:
        - cones: numpy array of cone data (x, y, class, ID).

        Returns:
        - left_boundary: list of cones representing the left boundary.
        - right_boundary: list of cones representing the right boundary.
        """
        left_boundary = [
            cone for id in self.left_boundary_ids for cone in cones if cone[3] == id
        ]
        right_boundary = [
            cone for id in self.right_boundary_ids for cone in cones if cone[3] == id
        ]
        return left_boundary, right_boundary

    def create_boundary_msg(self, boundary, header):
        """
        Creates a ROS Path message from a list of boundary points.

        Parameters:
        - boundary: list of cones representing a boundary.
        - header: ROS message header to use for the path.

        Returns:
        - boundary_msg: Path message containing boundary poses.
        """
        boundary_msg = Path()
        boundary_msg.header = header
        for cone in boundary:
            pose = PoseStamped()
            pose.pose.position.x = cone[0] if self.closed else cone.x
            pose.pose.position.y = cone[1] if self.closed else cone.y
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            pose.header = header
            boundary_msg.poses.append(pose)
        return boundary_msg

    def publish_boundaries(self, boundaries, left_boundary_msg, right_boundary_msg):
        """
        Publishes the combined boundary message and separate left and right boundary messages.

        Parameters:
        - boundaries: Boundaries message containing left and right boundaries.
        - left_boundary_msg: Path message for the left boundary.
        - right_boundary_msg: Path message for the right boundary.
        """
        self.boundary_pub.publish(boundaries)
        self.left_boundary_pub.publish(left_boundary_msg)
        self.right_boundary_pub.publish(right_boundary_msg)

    def create_path_msg(self, local_path_with_ids, header):
        """
        Creates a PathWithIds message for the calculated path between boundaries.

        Parameters:
        - local_path_with_ids: list of points representing the path with boundary IDs.
        - header: header for the path.

        Returns:
        - local_path_msg: PathWithIds message containing the path poses with boundary IDs.
        """
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
        return local_path_msg


BoundaryEstimation()
