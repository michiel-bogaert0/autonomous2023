#!/usr/bin/env python3

import numpy as np
import rospy
import tf2_ros as tf
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
        self.set_health(2, "jnfeds", [])

        """
        Receives pathplanning path and smooths it using a BSpline
        """
        try:
            msg_frame_id = msg.header.frame_id

            mission = rospy.get_param("/mission", "")

            path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

            # Determine path closure and BSpline periodicity
            away_from_start = False
            per = 0  # BSpline periodicity, 0 = not periodic, 1 = periodic

            for node in path:
                distance_lc = (path[0, 0] - node[0]) ** 2 + (path[0, 1] - node[1]) ** 2
                if away_from_start and (
                    distance_lc < self.max_distance_away_from_start
                ):
                    per = 1
                    break
                if not away_from_start and (
                    distance_lc > self.min_distance_away_from_start
                ):
                    away_from_start = True

            # Transform closed path to world frame
            # and shift it away from car
            # to avoid jumping of path close to car
            if per and (
                mission == AutonomousMission.TRACKDRIVE
                or mission == AutonomousMission.AUTOCROSS
            ):
                trans = self.tf_buffer.lookup_transform(
                    self.world_frame, msg.header.frame_id, msg.header.stamp
                )
                transformed_msg = ROSNode.do_transform_path(msg, trans)

                msg_frame_id = self.world_frame

                path = np.array(
                    [
                        [p.pose.position.x, p.pose.position.y]
                        for p in transformed_msg.poses
                    ]
                )

                # Find point closest to 0, 0
                closest_point = np.argmin(np.sum(path**2, axis=1))

                # Shift start of path to origin - 10 points so that car never reachs start/stop of path
                path = np.roll(path, -closest_point - 10, axis=0)

            # Add zero pose to path if no closure of path
            if not per and (
                mission == AutonomousMission.TRACKDRIVE
                or mission == AutonomousMission.AUTOCROSS
            ):
                path = np.vstack(([0, 0], path))

            # Linear interpolation between center points to add more points for BSpline smoothing
            distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
            distance = np.insert(distance, 0, 0) / distance[-1]

            alpha = np.linspace(0, 1, len(path) * 3)
            interpolator = interp1d(distance, path, kind="linear", axis=0)
            path = interpolator(alpha)

            # Smooth path with BSpline interpolation
            # Transpose to get correct shape for BSpline, splprep expects (2, N)
            path = path.T

            # Weights for BSpline
            w = np.ones(len(path[0]))

            # Calculate smoothing Spline
            tck, u = splprep(path, w=w, s=1, per=per)

            # Evaluate BSpline and transpose back to (N, 2)
            smoothed_path = np.array(splev(u, tck)).T
            vis_path = smoothed_path

            if per and (
                mission == AutonomousMission.TRACKDRIVE
                or mission == AutonomousMission.AUTOCROSS
            ):
                # Throw away last point to avoid weird FWF bug (see wiki)
                smoothed_path = smoothed_path[:-1]

            smoothed_msg = msg
            smoothed_msg.header.frame_id = msg_frame_id
            smoothed_msg.poses = []
            for point in smoothed_path:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                smoothed_msg.poses.append(pose)

            self.publisher.publish(smoothed_msg)

            vis_msg = msg
            vis_msg.header.frame_id = msg_frame_id
            vis_msg.poses = []
            for point in vis_path:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                vis_msg.poses.append(pose)

            self.publisher_vis_path.publish(vis_msg)
        except Exception as e:
            rospy.logerr("Error occurred while smoothing PoseArray: {}".format(e))
            self.publisher.publish(msg)


PathSmoother()
