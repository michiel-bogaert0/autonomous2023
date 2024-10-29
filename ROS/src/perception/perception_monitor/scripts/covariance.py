#!/usr/bin/env python3

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped


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
        self.match_threshold = rospy.get_param("~match_distance", 0.3)
        self.batch_size = rospy.get_param("~batch_size", 10)
        self.convergence_threshold = rospy.get_param("~convergence_threshold", 0.01)
        self.gt_x, self.gt_y, self.gt_z = (
            rospy.get_param("~ground_truth_x", 0.0),
            rospy.get_param("~ground_truth_y", 0.0),
            rospy.get_param("~ground_truth_z", 0.0),
        )

        self.lidar_obs_received = 0
        self.lidar_observations = [[], [], []]
        self.lidar_avg_x, self.lidar_avg_y, self.lidar_avg_z = 0.0, 0.0, 0.0
        self.lidar_covariance_matrix = np.ones((3, 3))
        self.converged = False

        self.camera_observations = [[], [], []]
        self.camera_obs_received = 0

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
            ) = np.average(self.lidar_observations, axis=1)
            x, y, z = (
                observation.observation.location.x,
                observation.observation.location.y,
                observation.observation.location.z,
            )
            distance_to_gt = np.linalg.norm(
                [x - self.gt_x, y - self.gt_y, z - self.gt_z]
            )

            # If the distance to the ground truth is less than the match threshold or there are too few observations,
            # add the observation to the list
            if (
                distance_to_gt < self.match_threshold
                or self.lidar_obs_received <= self.batch_size
            ):
                self.lidar_obs_received += 1
                self.lidar_observations[0].append(x)
                self.lidar_observations[1].append(y)
                self.lidar_observations[2].append(z)

            # Calculate covariance matrix every n=batch_size observations (starting from 2 or more observations)
            if self.lidar_obs_received % self.batch_size == 2:
                new_covariance_matrix = np.cov(self.lidar_observations)

                # Check for convergence of the covariance_matrix
                if (
                    np.min(self.lidar_covariance_matrix - new_covariance_matrix)
                    < self.convergence_threshold
                ):
                    self.converged = True
                self.lidar_covariance_matrix = new_covariance_matrix

                # Create and publish diagnostics
                diag_msg = DiagnosticStatus()
                diag_msg.name = "Lidar Covariance"
                diag_msg.message = f"Converged: {self.converged}"
                diag_msg.values.append(
                    KeyValue("num_observations", str(self.lidar_obs_received))
                )
                diag_msg.values.append(
                    KeyValue(
                        "cone_distance (2D)",
                        str(np.linalg.norm([self.lidar_avg_x, self.lidar_avg_y])),
                    )
                )
                diag_msg.values.append(
                    KeyValue("covariance_matrix", str(self.lidar_covariance_matrix))
                )
                diag_msg.values.append(
                    KeyValue("varX", str(self.lidar_covariance_matrix[0][0]))
                )
                diag_msg.values.append(
                    KeyValue("varY", str(self.lidar_covariance_matrix[1][1]))
                )
                diag_msg.values.append(
                    KeyValue("varZ", str(self.lidar_covariance_matrix[2][2]))
                )
                rospy.loginfo(self.lidar_covariance_matrix)
                rospy.loginfo("\n")
                rospy.loginfo(self.lidar_obs_received)
                self.publish(diag_msg)

    def camera_callback(self, observations: ObservationWithCovarianceArrayStamped):
        for observation in observations.observations:
            (
                self.camera_avg_x,
                self.camera_avg_y,
                self.camera_avg_z,
            ) = np.average(self.camera_observations, axis=1)
            x, y, z = (
                observation.observation.location.x,
                observation.observation.location.y,
                observation.observation.location.z,
            )
            distance_to_gt = np.linalg.norm(
                [x - self.gt_x, y - self.gt_y, z - self.gt_z]
            )

            # If the distance to the ground truth is less than the match threshold or there are too few observations,
            # add the observation to the list
            if (
                distance_to_gt < self.match_threshold
                or self.camera_obs_received <= self.batch_size
            ):
                self.camera_obs_received += 1
                self.camera_observations[0].append(x)
                self.camera_observations[1].append(y)
                self.camera_observations[2].append(z)

            # Calculate covariance matrix every n=batch_size observations (starting from 2 or more observations)
            if self.camera_obs_received % self.batch_size == 2:
                new_covariance_matrix = np.cov(self.camera_observations)

                # Check for convergence of the covariance_matrix
                if (
                    np.min(self.camera_covariance_matrix - new_covariance_matrix)
                    < self.convergence_threshold
                ):
                    self.converged = True
                self.camera_covariance_matrix = new_covariance_matrix

                # Create and publish diagnostics
                diag_msg = DiagnosticStatus()
                diag_msg.name = "Camera Covariance"
                diag_msg.message = f"Converged: {self.converged}"
                diag_msg.values.append(
                    KeyValue("num_observations", str(self.camera_obs_received))
                )
                diag_msg.values.append(
                    KeyValue(
                        "cone_distance (2D)",
                        str(np.linalg.norm([self.camera_avg_x, self.camera_avg_y])),
                    )
                )
                diag_msg.values.append(
                    KeyValue("covariance_matrix", str(self.camera_covariance_matrix))
                )
                diag_msg.values.append(
                    KeyValue("varX", str(self.camera_covariance_matrix[0][0]))
                )
                diag_msg.values.append(
                    KeyValue("varY", str(self.camera_covariance_matrix[1][1]))
                )
                diag_msg.values.append(
                    KeyValue("varZ", str(self.camera_covariance_matrix[2][2]))
                )
                rospy.loginfo(self.camera_covariance_matrix)
                rospy.loginfo("\n")
                rospy.loginfo(self.camera_obs_received)
                self.publish(diag_msg)


node = CovarianceNode()
rospy.spin()
