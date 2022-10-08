#!/usr/bin/env python3
import copy

import numpy as np
import rospy
from fs_msgs.msg import Track
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import AddSubscriber, DataLatch, ROSNode
from StageSimulator import StageSimulator
from tf.transformations import euler_from_quaternion
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class PerceptionSimulator(StageSimulator):
    def __init__(self) -> None:
        """
        This node simulates the perception stage by publishing cone "observations".
        """

        self.datalatch = DataLatch()

        self.datalatch.create("cones", 1)
        self.datalatch.create("odom", 400)

        super().__init__("perception")

        self.world_frame = rospy.get_param("~world_frame", "ugr/car_odom")
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.viewing_distance = rospy.get_param("~viewing_distance", 12.0) ** 2
        self.fov = np.deg2rad(rospy.get_param("~fov", 60))
        self.add_noise = rospy.get_param("~add_noise", True)
        self.cone_noise = rospy.get_param("~cone_noise", 0.2)
        self.publish_delay = rospy.get_param("~publish_delay", 0.5)

    @AddSubscriber("/input/track")
    def track_update(self, track: Track):
        """
        Track update is used to collect the ground truth cones
        """
        cones = []
        for cone in track.track:
            cones.append(
                np.array(
                    [cone.location.x, cone.location.y, cone.location.z, cone.color],
                    dtype=np.float32,
                )
            )

        cones = np.array(cones)

        self.datalatch.set("cones", cones)

    @AddSubscriber("/input/odometry")
    def odom_update(self, odom: Odometry):
        """
        Odometry is used to be able to decide which cones are 'visible'
        """
        self.datalatch.set("odom", odom)

    def simulate(self, timer):
        now = self.convertROSStampToTimestamp(rospy.get_rostime())

        odometry_updates = self.datalatch.get("odom")

        # Check if there is an update old enough
        if (
            len(odometry_updates) == 0
            or now
            - ROSNode.convertROSStampToTimestamp(odometry_updates[0].header.stamp)
            < self.publish_delay
        ):
            return

        odom = odometry_updates.popleft()

        # Try to find the odom update with the right delay
        # TODO Should be done with the tf package!
        while (
            len(odometry_updates) != 0
            and now - ROSNode.convertROSStampToTimestamp(odom.header.stamp)
            > self.publish_delay
        ):
            now = ROSNode.convertROSStampToTimestamp(rospy.get_rostime())
            odom = odometry_updates.popleft()

        pos: Point = odom.pose.pose.position
        orient: Quaternion = odom.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [orient.x, orient.y, orient.z, orient.w]
        )

        new_cones = copy.deepcopy(self.datalatch.get("cones"))
        new_cones[:, :-2] = PerceptionSimulator.apply_transformation(
            new_cones[:, :-2], [pos.x, pos.y], yaw, True
        )

        if self.add_noise:
            new_cones[:, :2] += (
                1 / 2 * np.random.randn(new_cones.shape[0], 2) * self.cone_noise
            )
        filtered_cones = []
        for cone in new_cones:
            if (
                cone[0] ** 2 + cone[1] ** 2 + cone[2] ** 2
            ) > self.viewing_distance or abs(
                self.pi_2_pi(np.arctan2(cone[1], cone[0]))
            ) > self.fov:
                continue

            filtered_cones.append(
                ObservationWithCovariance(
                    location=Point(x=cone[0], y=cone[1], z=cone[2]),
                    observation_class=int(cone[3]),
                )
            )

        observations = ObservationWithCovarianceArrayStamped()
        observations.header.stamp = odom.header.stamp
        observations.header.frame_id = self.base_link_frame
        observations.observations = filtered_cones

        self.publish("/output/observations", observations)

    @staticmethod
    def pi_2_pi(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def get_rotation_matrix(yaw):
        """
        Gets a rotation matrix based on angle yaw (must be in radians!)
        """
        R = np.array([[np.cos(yaw), -1 * np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        return R

    @staticmethod
    def apply_transformation(points, translation, yaw, inverse=False):
        """
        Basically applies a transformation to a set of points. First rotation, then translation
        """
        if inverse:
            R = PerceptionSimulator.get_rotation_matrix(-1 * yaw)
            tf_points = np.array(points)
            tf_points[:, 0] -= translation[0]
            tf_points[:, 1] -= translation[1]
            tf_points = R.dot(np.array(tf_points).T).T

        else:
            R = PerceptionSimulator.get_rotation_matrix(yaw)
            tf_points = R.dot(np.array(points).T).T
            tf_points[:, 0] += translation[0]
            tf_points[:, 1] += translation[1]

        return tf_points


node = PerceptionSimulator()
node.start()
