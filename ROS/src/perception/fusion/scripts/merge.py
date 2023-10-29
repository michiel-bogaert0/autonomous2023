#!/usr/bin/env python3
import rospy
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped

import time


class MergeNode:
    
    def publish(self, msg):
        """
        Just publishes on the topic
        """
        #rospy.logwarn(msg)
        self.result_publisher.publish(msg)
    
    def process_lidar_observations(
        self, observations: ObservationWithCovarianceArrayStamped
    ):
        """
        Process incoming lidar observations
        """

        timestamp_lidar = float(observations.header.stamp.secs) + float(observations.header.stamp.nsecs)*1e-9
        self.plot_observations_received(observations)
    
    def process_camera_observations(
        self, observations: ObservationWithCovarianceArrayStamped
    ):
        """
        Process incoming camera observations
        """

        timestamp_cam = float(observations.header.stamp.secs) + float(observations.header.stamp.nsecs)*1e-9
        self.plot_observations_received(observations)

    def plot_observations_received(self, observations: ObservationWithCovarianceArrayStamped):
        time_now = time.time_ns()*1e-6
        rospy.logwarn(f"\nTime since\n    lidar: {time_now-self.lidar_obs} ms;\n    camera: {time_now-self.camera_obs} ms;\nName: {observations.header.frame_id}; Time: {time_now};\n\n")
        if observations.header.frame_id == "os_sensor":
            self.lidar_obs = time_now
        else:
            self.camera_obs = time_now




    def __init__(self):
        """
        Subscribes to two Observation topics and
        """
        rospy.init_node("observation_merger_node", anonymous=True)

        rospy.Subscriber(
            "/input/lidar_observations", ObservationWithCovarianceArrayStamped, self.process_lidar_observations
        )
        rospy.Subscriber(
            "/input/camera_observations", ObservationWithCovarianceArrayStamped, self.process_camera_observations
        )

        self.result_publisher = rospy.Publisher(
            "/output/topic", ObservationWithCovarianceArrayStamped, queue_size=10
        )

        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.camera_obs, self.lidar_obs = 0.0, 0.0


node = MergeNode()
rospy.spin()
