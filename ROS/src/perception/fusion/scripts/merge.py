#!/usr/bin/env python3

import rospy
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped
from naive_fusion import NaiveFusion
from node_fixture.fixture import ROSNode
from ugr_msgs.msg import (  # ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class MergeNode:
    def __init__(self):
        """
        Subscribes to two Observation topics, sets up a publisher and initializes parameters and variables
        """
        rospy.init_node("observation_merger_node", anonymous=True)

        # Set up subscribers and publisher to receive/send observations
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
        #   Frames
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")

        #   Topics
        self.lidar_input_topic, self.camera_input_topic = (
            "/input/lidar_observations",
            "/input/camera_observations",
        )
        self.lidar_sensor_name, self.camera_sensor_name = (
            "os_sensor",
            "ugr/car_base_link/cam0",
        )
        self.input_sensors = ["os_sensor", "ugr/car_base_link/cam0"]

        #   Fusion parameters
        self.max_fusion_eucl_distance = rospy.get_param("~fusion_eucl_distance", 2.5)
        self.max_sensor_time_diff = rospy.get_param("~sensor_time_diff_ms", 50)

        #   Initialize fusion pipeline
        self.fusion_pipeline = None
        self.fusion_method = rospy.get_param("~fusion_method", "naive")

        if self.fusion_method == "naive":
            self.fusion_pipeline = NaiveFusion(self.max_fusion_eucl_distance)

        # Helper variables
        self.last_received = 0
        self.is_first_received = True
        self.msg_buffer = []
        self.sensors_received = []
        self.msg_wait_timer = None

    def publish(self, msg):
        """
        Just publishes on the topic
        """
        self.result_publisher.publish(msg)

    def handle_observations(self, observations: ObservationWithCovarianceArrayStamped):
        """
        Handle incoming observations and send through pipeline
        Wait for either all messages specified in self.input_sensors to arrive or for a timeout,
        then send all received messages through the fusion pipeline
        """
        self.msg_buffer.append(observations)
        self.sensors_received.append(observations.header.frame_id)

        if self.is_first_received:
            self.is_first_received = False
            self.msg_wait_timer = rospy.Timer(
                period=rospy.Duration(self.max_sensor_time_diff * 1e-3),
                callback=self.handle_timeout,
                oneshot=True,
            )

        # Check if messages from all sensors have been received
        all_received = True
        for sensor in self.input_sensors:
            if sensor not in self.sensors_received:
                all_received = False
                break

        # If all messages have been received, send through pipeline
        if all_received:
            self.msg_wait_timer.shutdown()
            rospy.loginfo("All msgs received")
            self.handle_timeout()
            return
        return

    def handle_timeout(self, event=None):
        self.sensors_received = []
        self.is_first_received = True
        send_msgs = self.msg_buffer.copy()
        self.msg_buffer = []

        rospy.loginfo("\n\n * Send through fusion pipeline:\n")
        for sensormsg in send_msgs:
            rospy.loginfo(f"    * {sensormsg.header.frame_id}\n")
        if len(send_msgs) >= 1:
            self.run_fusion(send_msgs)
        return

    def run_fusion(self, sensor_msgs):
        """
        Process incoming messages through fusion pipeline
        """

        # Transform all observations to a common frame and time
        transformed_msgs = self.transform_observations(sensor_msgs)

        # Fuse transformed observations
        results = self.fusion_pipeline.fuse_observations(transformed_msgs)

        self.log_plot_info(sensor_msgs, results)
        # self.publish(results)
        return

    def transform_observations(self, sensor_msgs):
        """
        Transform all sensor observations to a common frame (self.base_link_frame)
        and time (timestamp of observation last received)
        """

        try:
            transformed_msgs = []
            tf_source_time = sensor_msgs[
                0
            ].header.stamp  # Time to which observations messages are transformed

            for sensor_msg in sensor_msgs:
                transform: TransformStamped = self.tf_buffer.lookup_transform_full(
                    target_frame=sensor_msg.header.frame_id,  # Transform from this frame,
                    target_time=sensor_msg.header.stamp,  # at this time ...
                    source_frame=self.base_link_frame,  # ... to this frame,
                    source_time=tf_source_time,  # at this time.
                    fixed_frame=self.world_frame,  # Frame that does not change over time, in this case the "/world" frame
                    timeout=rospy.Duration(0),  # Time-out
                )
                tf_sensor_msg: ObservationWithCovarianceArrayStamped = (
                    ROSNode.do_transform_observations(sensor_msg, transform)
                )
                transformed_msgs.append(tf_sensor_msg)

            return transformed_msgs

        except Exception as e:
            rospy.logerr(
                f"Mergenode has caught an exception during transformation. Exception: {e}"
            )
        return []

    def log_plot_info(self, msgs, fused_obs):
        """ """
        # Create list of sensors, observations & points of all incoming messages
        all_observations = []
        for msg in msgs:
            all_observations.extend(msg.observations)
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
        centers = list(
            map(
                lambda obs: [
                    obs.observation.location.x,
                    obs.observation.location.y,
                    obs.observation.location.z,
                ],
                fused_obs.observations,
            )
        )

        # Print location of all observations, predicted colors and fused observation locations
        rospy.loginfo("\n\n\n***\nPlot data:\n")
        rospy.loginfo(f"\npoints = {all_points}")
        rospy.loginfo(
            f"\ncolors = {[observation.observation.observation_class for observation in all_observations]}"
        )
        rospy.loginfo(f"\ncenters = {centers}")


node = MergeNode()
rospy.spin()
