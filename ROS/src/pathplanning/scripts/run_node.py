#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from node_fixture.node_fixture import AddSubscriber, ROSNode
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from ugr_msgs.msg import Observation, Observations

from pathplanning.rrt import Rrt
from pathplanning.triangulator import Triangulator

# Source: https://gitlab.msu.edu/av/av_notes/-/blob/master/ROS/Coordinate_Transforms.md
class TransformFrames:
    def __init__(self):
        """Create a buffer of transforms and update it with TransformListener."""
        self.tfBuffer = tf2_ros.Buffer()  # Creates a frame buffer
        tf2_ros.TransformListener(
            self.tfBuffer
        )  # TransformListener fills the buffer as background task

    def get_transform(self, source_frame, target_frame):
        """Lookup latest transform between source_frame and target_frame from the buffer."""
        try:
            trans = self.tfBuffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(0.2)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(
                f"Cannot find transformation from {source_frame} to {target_frame}"
            )
            raise Exception(
                f"Cannot find transformation from {source_frame} to {target_frame}"
            ) from e
        return trans  # Type: TransformStamped

    def pose_transform(self, pose_array: PoseArray) -> PoseArray:
        """Transform PoseArray to other frame.

        Args:
            pose_array: will be transformed to target_frame
        """
        target_frame = rospy.get_param("~output_frame", "odom")
        trans = self.get_transform(pose_array.header.frame_id, target_frame)
        new_header = Header(frame_id=target_frame, stamp=pose_array.header.stamp)
        pose_array_transformed = PoseArray(header=new_header)
        for pose in pose_array.poses:
            pose_s = PoseStamped(pose=pose, header=pose_array.header)
            pose_t = tf2_geometry_msgs.do_transform_pose(pose_s, trans)
            pose_array_transformed.poses.append(pose_t.pose)
        return pose_array_transformed

    def get_frame_A_origin_frame_B(self, frame_A, frame_B):
        """Returns the pose of the origin of frame_A in frame_B as a PoseStamped."""
        header = Header(frame_id=frame_A, stamp=rospy.Time(0))
        origin_A = Pose(
            position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
        )
        origin_A_stamped = PoseStamped(pose=origin_A, header=header)
        pose_frame_B = tf2_geometry_msgs.do_transform_pose(
            origin_A_stamped, self.get_transform(frame_A, frame_B)
        )
        return pose_frame_B


class PathPlanning(ROSNode):
    """Path planning node. Calculates and publishes path based on observations."""

    def __init__(self) -> None:
        """Initialize node"""
        super().__init__("exploration_mapping", False)

        self.frametf = TransformFrames()

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

        self.pub = rospy.Publisher("/output/path", PoseArray, queue_size=10)

        AddSubscriber("/input/local_map", 1)(self.receive_new_map)
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
        path = self.algorithm.get_path(self.cones)

        if path is None:
            rospy.loginfo("No path found")
            return
            
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
        output.header.stamp = header.stamp

        output_transformed = self.frametf.pose_transform(output)

        self.pub.publish(output_transformed)


node = PathPlanning()
node.start()
