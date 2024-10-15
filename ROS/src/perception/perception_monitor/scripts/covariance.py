#!/usr/bin/env python3

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class CovarianceNode:
    def __init__(self):
        """
        Subscribes to two Observation topics, sets up a publisher and initializes parameters and variables
        """
        rospy.init_node("sensor_covariance_node", anonymous=True)

        # Set up subscribers and publisher to receive/send observations
        rospy.Subscriber(
            "/input/lidar_observations",
            ObservationWithCovarianceArrayStamped,
            self.lidar_callback,
        )
        rospy.Subscriber(
            "/input/camera_observations",
            ObservationWithCovarianceArrayStamped,
            self.camera_callback,
        )

        self.result_publisher = rospy.Publisher(
            "/output/topic", DiagnosticStatus, queue_size=1
        )

        # Initialize variables
        self.use_lidar = rospy.get_param("~measure_lidar", False)
        self.use_camera = rospy.get_param("~measure_camera", False)
        self.match_threshold = rospy.get_param("~match_distance", 0.3)
        self.fp_threshold = rospy.get_param("~fp_distance", 1.0)
        self.batch_size = rospy.get_param("~batch_size", 10)
        self.convergence_threshold = rospy.get_param("~convergence_threshold", 0.01)

        self.lidar_obs_received = 0
        self.lidar_observations = {"x": [], "y": [], "z": []}
        self.lidar_avg_x, self.lidar_avg_y, self.lidar_avg_z = 0, 0, 0
        self.gt_x, self.gt_y, self.gt_z = 0.019323348999023, -0.02572094462811947, -0.860053539276123
        self.covariance_matrix = np.ones((3, 3))
        self.converged = False

        self.camera_observations = {"x": [], "y": [], "z": []}

    def publish(self, msg):
        """
        Publishes on the topic
        """
        self.result_publisher.publish(msg)

    def lidar_callback(self, observations: ObservationWithCovarianceArrayStamped):
        for observation in observations.observations:
            (
                    self.lidar_avg_x,
                    self.lidar_avg_y,
                    self.lidar_avg_z,
            ) = self.calculate_average()
            x, y, z = (
                observation.observation.location.x,
                observation.observation.location.y,
                observation.observation.location.z,
            )
            
            distance_to_gt = np.linalg.norm(
                [x - self.gt_x, y - self.gt_y, z - self.gt_z]
            )
            if (
                distance_to_gt > self.fp_threshold
                and self.lidar_obs_received > self.batch_size
            ):
                self.detect_false_positive(observation)
            if (
                distance_to_gt < self.match_threshold
                or self.lidar_obs_received <= self.batch_size
            ):
                self.lidar_obs_received += 1
                self.lidar_observations["x"].append(x)
                self.lidar_observations["y"].append(y)
                self.lidar_observations["z"].append(z)

            if self.lidar_obs_received % self.batch_size == 2:
                new_covariance_matrix = self.calculate_covariance()
                if (
                    np.min(self.covariance_matrix - new_covariance_matrix)
                    < self.convergence_threshold
                ):
                    self.converged = True
                self.covariance_matrix = new_covariance_matrix

                # Create and publish diagnostics
                diag_msg = DiagnosticStatus()
                diag_msg.name = "Lidar Covariance"
                diag_msg.message = f"Converged: {self.converged}"
                diag_msg.values.append(
                    KeyValue("num_observations", str(self.lidar_obs_received))
                )
                diag_msg.values.append(
                    KeyValue("cone_distance (2D)", str(np.linalg.norm([self.lidar_avg_x, self.lidar_avg_y])))
                )
                diag_msg.values.append(
                    KeyValue("covariance_matrix", str(self.covariance_matrix))
                )
                diag_msg.values.append(
                    KeyValue("varX", str(self.covariance_matrix[0][0]))
                )
                diag_msg.values.append(
                    KeyValue("varY", str(self.covariance_matrix[1][1]))
                )
                diag_msg.values.append(
                    KeyValue("varZ", str(self.covariance_matrix[2][2]))
                )
                rospy.loginfo(self.covariance_matrix)
                rospy.loginfo("\n")
                rospy.loginfo(self.lidar_obs_received)
                self.publish(diag_msg)

    def camera_callback(self, observations: ObservationWithCovarianceArrayStamped):
        pass

    def calculate_average(self):
        """
        Calculates the average location (x, y, z) of the observations
        """
        return (
            np.average(self.lidar_observations["x"]),
            np.average(self.lidar_observations["y"]),
            np.average(self.lidar_observations["z"]),
        )

    def calculate_covariance(self):
        """
        Calculates the covariance matrix
        """
        return np.cov(
            [
                self.lidar_observations["x"],
                self.lidar_observations["y"],
                self.lidar_observations["z"],
            ]
        )

    def detect_false_positive(self, observation: ObservationWithCovariance):
        """
        Detects false positives
        """
        diag_msg = DiagnosticStatus()
        diag_msg.name = "Lidar False Positives"
        diag_msg.message = "False positive detected"

        distance = np.linalg.norm(
            [
                observation.observation.location.x,
                observation.observation.location.y,
                observation.observation.location.z,
            ]
        )
        (
            avg_x,
            avg_y,
            avg_z,
        ) = self.calculate_average()
        avg_distance = np.linalg.norm(
            [
                avg_x,
                avg_y,
                avg_z,
            ]
        )

        diag_msg.values.append(KeyValue("distance", str(distance)))
        diag_msg.values.append(KeyValue("average_distance", str(avg_distance)))
        self.publish(diag_msg)


node = CovarianceNode()
rospy.spin()
