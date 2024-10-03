#!/usr/bin/env python3

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
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
            "/output/topic", DiagnosticArray, queue_size=1
        )

        # Initialize variables
        self.use_lidar = rospy.get_param("~measure_lidar", False)
        self.use_camera = rospy.get_param("~measure_camera", False)
        self.lidar_observations = []
        self.camera_observations = []

        self.lidar_obs_received = 0

    def publish(self, msg):
        """
        Publishes on the topic
        """
        self.result_publisher.publish(msg)

    def lidar_callback(self, observations: ObservationWithCovarianceArrayStamped):
        self.lidar_obs_received += 1

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()
        diag_msg.status = []

        my_status = DiagnosticStatus()
        my_status.values.append(
            KeyValue("num_observations", str(self.lidar_obs_received))
        )
        diag_msg.status.append(my_status)

        self.publish(diag_msg)

    def camera_callback(self, observations: ObservationWithCovarianceArrayStamped):
        pass


node = CovarianceNode()
rospy.spin()
