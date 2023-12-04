#!/usr/bin/env python3
import time

import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped
from node_fixture.fixture import ROSNode
from sklearn.neighbors import KDTree
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class MergeNode:
    def __init__(self):
        """
        Subscribes to two Observation topics, sets up a publisher and initializes parameters and variables
        """
        rospy.init_node("observation_merger_node", anonymous=True)

        rospy.Subscriber(
            "/input/lidar_observations",
            ObservationWithCovarianceArrayStamped,
            self.handle_observations,
        )
        rospy.Subscriber(
            "/input/camera_observations",
            ObservationWithCovarianceArrayStamped,
            self.handle_observations,
        )

        self.result_publisher = rospy.Publisher(
            "/output/topic", ObservationWithCovarianceArrayStamped, queue_size=10
        )

        # Initialize buffer and listener for time transformation with tf2
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # Parameters
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.lidar_input_topic, self.camera_input_topic = (
            "/input/lidar_observations",
            "/input/camera_observations",
        )
        self.lidar_sensor_name, self.camera_sensor_name = (
            "os_sensor",
            "ugr/car_base_link/cam0",
        )

        self.waiting_time_ms = 50
        self.merge_distance_threshold = 2.5

        # Random helpers
        self.camera_last_obs_time, self.lidar_last_obs_time = 0.0, 0.0
        self.is_first_received = True
        self.last_received_sensor = None

    def publish(self, msg):
        """
        Just publishes on the topic
        """
        self.result_publisher.publish(msg)

    def detect_observations(self, observations: ObservationWithCovarianceArrayStamped):
        """
        Measure time between incoming lidar and camera observations, log to console
        """

        time_now = time.time_ns() * 1e-6
        rospy.loginfo(
            f"\n\nSensor: {observations.header.frame_id}\nTime since\n    lidar: {time_now - self.lidar_last_obs_time} ms;\n    camera: {time_now - self.camera_last_obs_time} ms;\n Time: {time_now};\n\n"
        )

        if observations.header.frame_id == self.lidar_sensor_name:
            self.lidar_last_obs_time = time_now
        else:
            self.camera_last_obs_time = time_now

    def handle_observations(self, observations: ObservationWithCovarianceArrayStamped):
        """
        Process incoming lidar and camera observations
        This function is called every time a message is received by one of the subscribers
        The following algorithm provides a fairly robust way of handling the observations without putting too much delays on the pipeline
        """

        # Log observations to console
        self.detect_observations(observations)

        # If the observations are the first ones received after processing the last cycle, a cycle is initiated
        if (
            self.is_first_received
            or observations.header.frame_id == self.last_received_sensor
        ):
            self.is_first_received = False
            self.last_received_sensor = observations.header.frame_id

            # Setup up waiting loop for the other sensor. When time-out is reached, second_observations is set to None
            if observations.header.frame_id == self.lidar_sensor_name:
                rospy.loginfo("\nReceived lidar, waiting for camera...\n")
                try:
                    second_observations = rospy.wait_for_message(
                        self.camera_input_topic,
                        ObservationWithCovarianceArrayStamped,
                        timeout=self.waiting_time_ms * 1e-3,
                    )
                except rospy.exceptions.ROSException:
                    second_observations = None
            elif observations.header.frame_id == self.camera_sensor_name:
                rospy.loginfo("\nReceived camera, waiting for lidar...\n")
                try:
                    second_observations = rospy.wait_for_message(
                        self.lidar_input_topic,
                        ObservationWithCovarianceArrayStamped,
                        timeout=self.waiting_time_ms * 1e-3,
                    )
                except rospy.exceptions.ROSException:
                    second_observations = None
            else:
                rospy.logerr("Could not recognize sensor name...")
                return
        # If the observations are of the second sensor received, the helper variables are set to their initial states.
        # This same sensor data is also received by the wait_for_message() function above
        else:
            self.is_first_received = True
            self.last_received_sensor = observations.header.frame_id
            return

        # If waiting time is exceeded, only the sensor observations of the first sensor are published (other sensor will start new cycle when received)
        if second_observations is None:
            rospy.logwarn(
                f"Did not find second observations, publishing {observations.header.frame_id}"
            )
            self.publish(observations)
            self.is_first_received = True
            return

        # Transform observations of both sensors (transformation of both coordinate frame and time)
        self.transform_observations(observations, second_observations)
        self.is_first_received = True

    def transform_observations(
        self,
        early_obs: ObservationWithCovarianceArrayStamped,
        late_obs: ObservationWithCovarianceArrayStamped,
    ):
        """
        Transform both lidar and camera observations to a common frame (self.base_link_frame)
        and time (timestamp of first observation received)
        """

        try:
            # Find transform for both sensors
            tf_early_to_base: TransformStamped = self.tf_buffer.lookup_transform_full(
                target_frame=early_obs.header.frame_id,  # Transform from this frame,
                target_time=early_obs.header.stamp,  # at this time ...
                source_frame=self.base_link_frame,  # ... to this frame,
                source_time=late_obs.header.stamp,  # at this time.
                fixed_frame=self.world_frame,  # Frame that does not change over time, in this case the "/world" frame
                timeout=rospy.Duration(0),  # Time-out (not specified -> )
            )

            tf_late_to_base: TransformStamped = self.tf_buffer.lookup_transform_full(
                target_frame=late_obs.header.frame_id,  # Idem
                target_time=late_obs.header.stamp,
                source_frame=self.base_link_frame,
                source_time=late_obs.header.stamp,
                fixed_frame=self.world_frame,
                timeout=rospy.Duration(0),
            )

            # Apply transformations to observations
            time_transformed_early_obs: ObservationWithCovarianceArrayStamped = (
                ROSNode.do_transform_observations(early_obs, tf_early_to_base)
            )
            time_transformed_late_obs: ObservationWithCovarianceArrayStamped = (
                ROSNode.do_transform_observations(late_obs, tf_late_to_base)
            )

            # Proceed fusion by matching lidar with camera observations
            self.kd_tree_merger(time_transformed_early_obs, time_transformed_late_obs)

        except Exception as e:
            rospy.logerr(
                f"Mergenode has caught an exception. Ignoring ObservationWithCovarianceArrayStamped messages... Exception: {e}"
            )

    def kd_tree_merger(self, observations1, observations2):
        """
        Link lidar observations with camera observations based on euclidean distance (with a KDTree)
        """
        all_observations = observations1.observations + observations2.observations
        all_points = list(
            map(
                lambda obs: [
                    obs.observation.location.x,
                    obs.observation.location.y,
                    obs.observation.location.z,
                ],
                all_observations,
            )
        )
        kdtree_all = KDTree(all_points)

        centers = []
        resulting_observations = []

        # Loop through observations
        for observation in all_observations:
            if observation in observations1.observations:
                this_sensor_observations = observations1
                other_sensor_observations = observations2
            else:
                this_sensor_observations = observations2
                other_sensor_observations = observations1

            # Use set of observations of other sensor + this observations to avoid linking observations of the same sensor
            current_obs_set = [observation] + other_sensor_observations.observations
            sensor = this_sensor_observations.header.frame_id

            points = list(
                map(
                    lambda obs: [
                        obs.observation.location.x,
                        obs.observation.location.y,
                        obs.observation.location.z,
                    ],
                    current_obs_set,
                )
            )
            kdtree_observation = KDTree(points)

            location = [
                observation.observation.location.x,
                observation.observation.location.y,
                observation.observation.location.z,
            ]
            distance, indices = kdtree_observation.query([location], k=2)
            index = indices[0][1]

            # Make KDTree for all points to avoid multiple use of one observation
            distance2, indices2 = kdtree_all.query([points[index]], k=2)
            index2 = indices2[0][1]

            # Make sure both found observations have each other as nearest neighbor, and only match observations if the distance is smaller then the given threshold
            if (
                all_points[index2] == location
                and distance[0][1] <= self.merge_distance_threshold
                and location not in centers
            ):
                if sensor == self.lidar_sensor_name:
                    lidar_observation = observation
                    camera_observation = other_sensor_observations.observations[
                        index - 1
                    ]
                else:
                    camera_observation = observation
                    lidar_observation = other_sensor_observations.observations[
                        index - 1
                    ]

                # Find observation that is at the center of linked observations (either euclidean average or with Kalman filter)
                center_observation = self.kalman_filter(
                    lidar_observation, camera_observation
                )
                center_location = [
                    center_observation.observation.location.x,
                    center_observation.observation.location.y,
                    center_observation.observation.location.z,
                ]
                if center_location not in centers:
                    centers.append(center_location)
                    resulting_observations.append(center_observation)

            # Add isolated cones to results
            elif location not in centers:
                centers.append(location)
                resulting_observations.append(observation)

        # Print location of all observations (both lidar and camera), predicted colors and fused observation locations
        rospy.loginfo(f"\npoints = {all_points}")
        rospy.loginfo(
            f"\ncolors = {[observation.observation.observation_class for observation in all_observations]}"
        )
        rospy.loginfo(f"\ncenters = {centers}")

        # Create and publish new ObservationWithCovarianceArrayStamped object with fusion data
        result = ObservationWithCovarianceArrayStamped()
        result.header.seq = observations1.header.seq
        result.header.stamp.secs, result.header.stamp.nsecs = (
            observations1.header.stamp.secs,
            observations1.header.stamp.nsecs,
        )
        result.header.frame_id = observations1.header.frame_id
        result.observations = resulting_observations
        self.publish(result)

    def euclidean_average(self, lidar_observation, camera_observation):
        """
        Find euclidean average of lidar and camera observations for fusion
        """
        average_observation = ObservationWithCovariance()
        average_observation.observation.observation_class = (
            lidar_observation.observation.observation_class
        )
        average_observation.observation.belief = lidar_observation.observation.belief
        average_observation.covariance = tuple(
            list(lidar_observation.covariance).copy()
        )

        lidar_observation_location = [
            lidar_observation.observation.location.x,
            lidar_observation.observation.location.y,
            lidar_observation.observation.location.z,
        ]
        camera_observation_location = [
            camera_observation.observation.location.x,
            camera_observation.observation.location.y,
            camera_observation.observation.location.z,
        ]
        (
            average_observation.observation.location.x,
            average_observation.observation.location.y,
            average_observation.observation.location.z,
        ) = list(
            (
                np.array(lidar_observation_location)
                + np.array(camera_observation_location)
            )
            / 2
        )

        return average_observation

    def kalman_filter(self, lidar_observation, camera_observation):
        """
        This function provides a basic implementation of a Kalman filter to use for sensor fusion
        Mainly based on https://arxiv.org/pdf/1710.04055.pdf
        """

        # Initialize numpy arrays
        covariance_matrix_lidar = np.reshape(
            np.array(lidar_observation.covariance), (3, 3)
        )
        covariance_matrix_camera = np.reshape(
            np.array(camera_observation.covariance), (3, 3)
        )
        lidar_observation_location = np.reshape(
            np.array(
                [
                    lidar_observation.observation.location.x,
                    lidar_observation.observation.location.y,
                    lidar_observation.observation.location.z,
                ]
            ),
            (3, 1),
        )
        camera_observation_location = np.reshape(
            np.array(
                [
                    camera_observation.observation.location.x,
                    camera_observation.observation.location.y,
                    camera_observation.observation.location.z,
                ]
            ),
            (3, 1),
        )

        # Calcultate new covariance matrix
        new_covariance = np.linalg.inv(
            np.linalg.inv(covariance_matrix_lidar)
            + np.linalg.inv(covariance_matrix_camera)
        )
        # Calculate weights of each sensor's observations
        lidar_weight = np.matmul(new_covariance, np.linalg.inv(covariance_matrix_lidar))
        camera_weight = np.matmul(
            new_covariance, np.linalg.inv(covariance_matrix_camera)
        )

        # Calculate the weighted average of both observations based on lidar_weight and camera_weight
        new_prediction = np.matmul(
            lidar_weight, lidar_observation_location
        ) + np.matmul(camera_weight, camera_observation_location)

        fused_observation = ObservationWithCovariance()
        fused_observation.observation.observation_class = (
            camera_observation.observation.observation_class
        )
        fused_observation.observation.belief = camera_observation.observation.belief
        fused_observation.covariance = tuple(
            np.reshape(new_covariance, (1, 9)).tolist()[0]
        )
        (
            fused_observation.observation.location.x,
            fused_observation.observation.location.y,
            fused_observation.observation.location.z,
        ) = (new_prediction[0][0], new_prediction[1][0], new_prediction[2][0])

        return fused_observation


node = MergeNode()
rospy.spin()
