#!/usr/bin/env python3

import numpy as np
import rospy
import tf2_ros as tf
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture import AutonomousMission, ROSNode
from node_fixture.managed_node import ManagedNode
from scipy.interpolate import interp1d, splev, splprep


class PathSmoother(ManagedNode):
    def __init__(self):
        super().__init__("path_smoother")
        self.spin()

    def doConfigure(self):
        self.max_distance_away_from_start = rospy.get_param(
            "~max_distance_away_from_start", 9
        )
        self.min_distance_away_from_start = rospy.get_param(
            "~min_distance_away_from_start", 16
        )
        # make one parameter to check if trackdrive or autocross
        self.mission = rospy.get_param("/mission", "")
        self.trackdrive_autocross = (
            self.mission == AutonomousMission.TRACKDRIVE
            or self.mission == AutonomousMission.AUTOCROSS
        )

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.publisher = super().AddPublisher("/output/path", Path, queue_size=10)
        self.publisher_vis_path = super().AddPublisher(
            "/output/vis_path", Path, queue_size=10
        )

    def doActivate(self):
        self.subscriber = super().AddSubscriber(
            "/input/path", Path, self.pose_array_callback
        )

    def pose_array_callback(self, msg: Path):
        """
        Receives pathplanning path and smooths it using a BSpline
        """
        if msg is None:
            return
        try:
            self.process_path(msg)
        except Exception as e:
            self.set_health(
                DiagnosticStatus.WARN,
                message=f"Path smoother failed with error: '{e}' -> unsmoothed path is send!",
            )
            self.publisher.publish(msg)

    def process_path(self, msg):
        """
        Process the newest path, smooth it and publish it
        """
        msg_frame_id = msg.header.frame_id

        path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

        # Check if path is closed to determine if BSpline should be periodic
        closed_path = self.check_closed_path(path)

        # shift path in world frame to avoid jumps in smoothed path, see wiki (Path Smoother)
        if closed_path and self.trackdrive_autocross:
            path = self.transform_to_world_frame(msg)
            msg_frame_id = self.world_frame

            # Shift start of path to origin - 10 points so that car never reachs start/stop of path
            closest_point = np.argmin(np.sum(path**2, axis=1))
            path = np.roll(path, -closest_point - 10, axis=0)

            # add last point to path to have also interpolation between last and first point
            path = np.vstack((path, path[0]))

        # Add zero pose to path if no closure of path (force path starting from car)
        if not closed_path and self.trackdrive_autocross:
            path = np.vstack(([0, 0], path))

        # smooth path
        smoothed_path = self.smooth_path(path, closed_path)
        vis_path = smoothed_path

        # Throw away last point to avoid weird FWF bug, see wiki (Path Smoother)
        if closed_path and self.trackdrive_autocross:
            smoothed_path = smoothed_path[:-1]

        # Publish smoothed path
        smoothed_msg = msg
        smoothed_msg.header.frame_id = msg_frame_id
        smoothed_msg.poses = []
        for point in smoothed_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            smoothed_msg.poses.append(pose)

        self.publisher.publish(smoothed_msg)

        # Publish smoothed path for visualization
        vis_msg = msg
        vis_msg.header.frame_id = msg_frame_id
        vis_msg.poses = []
        for point in vis_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            vis_msg.poses.append(pose)

        self.publisher_vis_path.publish(vis_msg)

    def check_closed_path(self, path):
        """
        Check if path is closed to determine if BSpline should be periodic

        param path: Path to check
        return: True if path is closed, False otherwise
        """

        away_from_start = False
        closed_path = 0

        for node in path:
            distance_lc = (path[0, 0] - node[0]) ** 2 + (path[0, 1] - node[1]) ** 2
            if away_from_start and (distance_lc < self.max_distance_away_from_start):
                closed_path = 1
                break
            if not away_from_start and (
                distance_lc > self.min_distance_away_from_start
            ):
                away_from_start = True
        return closed_path

    def transform_to_world_frame(self, msg):
        """
        Transforms path to world frame
        """
        trans = self.tf_buffer.lookup_transform(
            self.world_frame, msg.header.frame_id, msg.header.stamp
        )
        transformed_path = ROSNode.do_transform_path(msg, trans)

        path = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in transformed_path.poses]
        )

        return path

    def smooth_path(self, path, closed_path):
        """
        Smooth path using BSpline interpolation

        param path: Path to smooth
        param closed_path: True if path is closed, False otherwise
        """

        # STEP1: Linear interpolation between center points to add more points for BSpline smoothing
        distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]

        alpha = np.linspace(0, 1, len(path) * 3)
        interpolator = interp1d(distance, path, kind="linear", axis=0)
        path = interpolator(alpha)

        # STEP2: Smooth path with BSpline interpolation
        # Transpose to get correct shape for BSpline, splprep expects (2, N)
        path = path.T
        # Weights for BSpline
        w = np.ones(len(path[0]))
        # Calculate smoothing Spline
        tck, u = splprep(path, w=w, s=1, per=closed_path)
        # Evaluate BSpline and transpose back to (N, 2)
        smoothed_path = np.array(splev(u, tck)).T

        return smoothed_path


PathSmoother()
