#!/usr/bin/env python3
# import time

# import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped
from naive_fusion import NaiveFusion
from node_fixture.node_fixture import ROSNode
from ugr_msgs.msg import (  # ObservationWithCovariance,
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
        #
        # Frames
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")

        # Topics
        self.lidar_input_topic, self.camera_input_topic = (
            "/input/lidar_observations",
            "/input/camera_observations",
        )
        self.lidar_sensor_name, self.camera_sensor_name = (
            "os_sensor",
            "ugr/car_base_link/cam0",
        )
        # Fusion parameters
        self.max_fusion_eucl_distance = rospy.get_param("~fusion_eucl_distance", 2.5)
        self.max_sensor_time_diff = rospy.get_param("~sensor_time_diff_ms", 50)

        # Initialize fusion pipeline
        self.fusion_pipeline = None
        self.fusion_method = rospy.get_param("~fusion_method", "naive")

        if self.fusion_method == "naive":
            self.fusion_pipeline = NaiveFusion(self.max_fusion_eucl_distance)

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
        return

    def handle_observations(self, observations: ObservationWithCovarianceArrayStamped):
        """
        Handle incoming observations and send through pipeline
        """

        self.run_fusion(observations)
        return

    def run_fusion(self, sensor_msgs):
        """
        Process incoming messages through fusion pipeline
        """

        # Transform all observations to a common frame and time
        transformed_msgs = self.transform_observations(sensor_msgs)

        # Fuse transformed observations
        results = self.fusion_pipeline.fuse_observations(transformed_msgs)
        self.publish(results)
        return

    def transform_observations(self, sensor_msgs):
        """
        Transform all sensor observations to a common frame (self.base_link_frame)
        and time (timestamp of observation last received)
        """

        try:
            transformed_msgs = []
            tf_source_time = sensor_msgs[
                -1
            ].header.stamp  # Time to which observations messages are transformed

            for sensor_msg in sensor_msgs:
                transform: TransformStamped = self.tf_buffer.lookup_transform_full(
                    target_frame=sensor_msg.header.frame_id,  # Transform from this frame,
                    target_time=sensor_msg.header.stamp,  # at this time ...
                    source_frame=self.base_link_frame,  # ... to this frame,
                    source_time=tf_source_time.header.stamp,  # at this time.
                    fixed_frame=self.world_frame,  # Frame that does not change over time, in this case the "/world" frame
                    timeout=rospy.Duration(0),  # Time-out
                )
                tf_sensor_msg: ObservationWithCovarianceArrayStamped = (
                    ROSNode.do_transform_observation(sensor_msg, transform)
                )
                transformed_msgs.append(tf_sensor_msg)

            return transformed_msgs

        except Exception as e:
            rospy.logerr(
                f"Mergenode has caught an exception during transformation. Exception: {e}"
            )


node = MergeNode()
rospy.spin()
