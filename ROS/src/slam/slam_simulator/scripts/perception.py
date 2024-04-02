#!/usr/bin/env python3
import copy

import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from node_fixture.fixture import (
    DataLatch,
    DiagnosticArray,
    DiagnosticStatus,
    create_diagnostic_message,
)
from StageSimulator import StageSimulator
from tf.transformations import euler_from_quaternion
from ugr_msgs.msg import (
    Observation,
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class PerceptionSimulator(StageSimulator):
    def __init__(self) -> None:
        """
        This node simulates the perception stage by publishing cone "observations".

        Args:
            world_frame
            base_link_frame
            viewing_distance: the radius around base_link_frame of cones that are 'visible'
            fov: the Field of View (fov/2 'to the left' and fov/2 'to the right') of cones that are 'visible'
            add_noise: If true, adds some noise, if false, it just publishes GT
            cone_noise: if add_noise is True, this is the standard deviation of the (Gaussian) noise source in meter
        """

        self.datalatch = DataLatch()

        self.datalatch.create("cones", 1)
        self.datalatch.create("odom", 400)

        self.started = False

        super().__init__("perception")

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.gt_base_link_frame = rospy.get_param(
            "~gt_base_link_frame", "ugr/gt_base_link"
        )
        self.viewing_distance = rospy.get_param("~viewing_distance", 15.0) ** 2
        self.fov = np.deg2rad(rospy.get_param("~fov", 90))
        self.delay = rospy.get_param("~delay", 0.0)
        self.add_noise = rospy.get_param("~add_noise", True)
        self.cone_noise = rospy.get_param(
            "~cone_noise", 0.0 / 20
        )  # Noise per meter distance. Gets scaled with range

        self.color_prob = rospy.get_param("~color_prob", 0.05)

        # Diagnostics Publisher
        self.diagnostics = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        # Publish Started Diagnostic
        self.diagnostics.publish(
            create_diagnostic_message(
                level=DiagnosticStatus.OK,
                name="[SLAM SIM] Perception Status",
                message="Started.",
            )
        )
        self.track_sub = rospy.Subscriber(
            "/input/track", ObservationWithCovarianceArrayStamped, self.track_update
        )
        self.observation_pub = rospy.Publisher(
            "/output/observations", ObservationWithCovarianceArrayStamped, queue_size=10
        )

    def track_update(self, track: ObservationWithCovarianceArrayStamped):
        """
        Track update is used to collect the ground truth cones
        """
        cones = []
        for cone in track.observations:
            cones.append(
                np.array(
                    [
                        cone.observation.location.x,
                        cone.observation.location.y,
                        cone.observation.location.z,
                        cone.observation.observation_class,
                    ],
                    dtype=np.float32,
                )
            )

        cones = np.array(cones)
        self.datalatch.set("cones", cones)

        self.started = True

    def simulate(self, _):
        """
        This function gets executed at a specific frequency. Basically publishes the 'visible' cones as observations
        Those observations are relative to the base_link frame
        """
        if not self.started:
            return

        try:
            # Fetch GT position of car
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.gt_base_link_frame,
                rospy.Time.now() - rospy.Duration(self.delay),
                rospy.Duration(0.2),
            )
        except Exception:
            return

        pos = transform.transform.translation
        orient: Quaternion = transform.transform.rotation
        roll, pitch, yaw = euler_from_quaternion(
            [orient.x, orient.y, orient.z, orient.w]
        )

        new_cones = copy.deepcopy(self.datalatch.get("cones"))

        new_cones[:, :-2] = PerceptionSimulator.apply_transformation(
            new_cones[:, :-2], [pos.x, pos.y], yaw, True
        )

        filtered_cones = []
        for cone in new_cones:
            if (
                cone[0] ** 2 + cone[1] ** 2 + cone[2] ** 2
            ) > self.viewing_distance or abs(
                self.pi_2_pi(np.arctan2(cone[1], cone[0]))
            ) > self.fov:
                continue

            range = (cone[0] ** 2 + cone[1] ** 2) ** (1 / 2)
            if self.add_noise:
                cone[0] += np.random.randn() * self.cone_noise * range
                cone[1] += np.random.randn() * self.cone_noise * range

            cov = [
                (self.cone_noise * range) ** 2,
                0.0,
                0.0,
                0.0,
                (self.cone_noise * range) ** 2,
                0.0,
                0.0,
                0.0,
                0.0,
            ]

            if cone[3] == 0 or cone[3] == 1:
                if np.random.rand() < self.color_prob:
                    cone[3] = 1 - cone[3]

            filtered_cones.append(
                ObservationWithCovariance(
                    observation=Observation(
                        belief=1.0,
                        location=Point(x=cone[0], y=cone[1], z=cone[2]),
                        observation_class=int(cone[3]),
                    ),
                    covariance=cov,
                )
            )

        observations = ObservationWithCovarianceArrayStamped()
        observations.header.stamp = transform.header.stamp
        observations.header.frame_id = self.base_link_frame
        observations.observations = filtered_cones

        self.observation_pub.publish(observations)

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
rospy.spin()
