#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped
import tf2_ros as tf
from node_fixture.node_fixture import ROSNode
from sklearn.neighbors import KDTree
import numpy as np

import time


class MergeNode:
    def __init__(self):
        """
        Subscribes to two Observation topics and
        """
        rospy.init_node("observation_merger_node", anonymous=True)

        rospy.Subscriber(
            "/input/lidar_observations", ObservationWithCovarianceArrayStamped, self.handle_observations
        )
        rospy.Subscriber(
            "/input/camera_observations", ObservationWithCovarianceArrayStamped, self.handle_observations
        )

        self.result_publisher = rospy.Publisher(
            "/output/topic", ObservationWithCovarianceArrayStamped, queue_size=10
        )

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # Parameters
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")

        self.lidar_input_topic, self.camera_input_topic = "/input/lidar_observations", "/input/camera_observations"
        self.lidar_sensor_name, self.camera_sensor_name = "os_sensor", "ugr/car_base_link/cam0"
        self.camera_last_obs_time, self.lidar_last_obs_time = 0.0, 0.0
        self.waiting_time_ms = 50

        self.merge_distance_threshold = 4.0

        # Random helpers
        self.is_first_received = True
        self.last_received_sensor = None
    

    def publish(self, msg):
        """
        Just publishes on the topic
        """
        rospy.loginfo(f"\npublishing {msg.header.frame_id}")
        self.result_publisher.publish(msg)
    
    def detect_observations(
        self, observations: ObservationWithCovarianceArrayStamped
    ):
        """
        Measure and display incoming lidar and camera observations
        """

        timestamp_cam = float(observations.header.stamp.secs) + float(observations.header.stamp.nsecs)*1e-9

        time_now = time.time_ns()*1e-6
        rospy.loginfo(f"\nTime since\n    lidar: {time_now-self.lidar_last_obs_time} ms;\n    camera: {time_now-self.camera_last_obs_time} ms;\nName: {observations.header.frame_id}; Time: {time_now};\n\n")
        if len(observations.observations) >= 1:
            rospy.loginfo(observations.observations[0])

        if observations.header.frame_id == self.lidar_sensor_name:
            self.lidar_last_obs_time = time_now
        else:
            self.camera_last_obs_time = time_now
    
    def handle_observations(
        self, observations: ObservationWithCovarianceArrayStamped
    ):
        """
        Process incoming lidar and camera observations
        """
        self.detect_observations(observations)
        if self.is_first_received or observations.header.frame_id == self.last_received_sensor:
            self.is_first_received = False
            self.last_received_sensor = observations.header.frame_id

            if observations.header.frame_id == self.lidar_sensor_name:
                rospy.loginfo(f"\nReceived lidar, waiting for camera...\n")
                try:
                    second_observations = rospy.wait_for_message(self.camera_input_topic, ObservationWithCovarianceArrayStamped, timeout=self.waiting_time_ms*1e-3)
                except rospy.exceptions.ROSException:
                    second_observations = None
            elif observations.header.frame_id == self.camera_sensor_name:
                rospy.loginfo(f"\nReceived camera, waiting for lidar...\n")
                try:
                    second_observations = rospy.wait_for_message(self.lidar_input_topic, ObservationWithCovarianceArrayStamped, timeout=self.waiting_time_ms*1e-3)
                except rospy.exceptions.ROSException:
                    second_observations = None
            else:
                rospy.logerr("Could not recognize sensor name")
                return
        else:
            self.is_first_received = True
            self.last_received_sensor = observations.header.frame_id
            return
        
        if second_observations == None:
            rospy.logwarn(f"Did not find second observations, publishing {observations.header.frame_id}")
            self.publish(observations)
            self.is_first_received = True
            return
        
        self.transform_observations(observations, second_observations)
        self.is_first_received = True

    def transform_observations(
        self, early_obs: ObservationWithCovarianceArrayStamped, late_obs: ObservationWithCovarianceArrayStamped
    ):
        """
        * * * FUNCTION STILL INCOMPLETE (AND PROBABLY INCORRECT)! * * *
        """

        try:
            tf_early_to_base: TransformStamped = self.tf_buffer.lookup_transform_full(
                    target_frame=early_obs.header.frame_id,         # Give the transform from this frame,
                    target_time=early_obs.header.stamp,             # at this time ...
                    source_frame=self.base_link_frame,              # ... to this frame,
                    source_time=late_obs.header.stamp,              # at this time.
                    fixed_frame=self.world_frame                    # Specify the frame that does not change over time, in this case the "/world" frame, and
                                                                    #the time-out
                )
            
            tf_late_to_base: TransformStamped = self.tf_buffer.lookup_transform_full(
                    target_frame=late_obs.header.frame_id,          # Give the transform from this frame,
                    target_time=late_obs.header.stamp,              # at this time ...
                    source_frame=self.base_link_frame,              # ... to this frame,
                    source_time=late_obs.header.stamp,              # at this time.
                    fixed_frame=self.world_frame                    # Specify the frame that does not change over time, in this case the "/world" frame, and
                                                                    #the time-out
                )
            
            
            time_transformed_early_obs = ROSNode.do_transform_observations(
                    early_obs, tf_early_to_base
            )
            time_transformed_late_obs = ROSNode.do_transform_observations(
                    late_obs, tf_late_to_base
            )
            
            self.kd_tree_merger(time_transformed_early_obs, time_transformed_late_obs)


        except Exception as e:
            rospy.logerr(
                f"Mergenode has caught an exception. Ignoring ObservationWithCovarianceArrayStamped messages... Exception: {e}"
            )

    def kd_tree_merger(self, observations1, observations2):
        all_observations = observations1.observations + observations2.observations
        points = list(map(lambda obs: [obs.observation.location.x, obs.observation.location.y, obs.observation.location.z], all_observations))
        kdtree = KDTree(points)

        centers = []

        for observation in all_observations:
            location = [observation.observation.location.x, observation.observation.location.y, observation.observation.location.z]
            distance, indices = kdtree.query([location], k=2)
            index = indices[0][1]

            distance2, indices2 = kdtree.query([points[index]], k=2)
            index2 = indices2[0][1]

            if points[index2] == location and distance[0][1] <= self.merge_distance_threshold:
                location2 = points[index]
                center = list((np.asarray(location) + np.asarray(location2))/2)
                if center not in centers:
                    centers.append(center)
                rospy.loginfo(f"\nCenter found at: {center};\n\n")
            elif location not in centers:
                centers.append(location)


        rospy.loginfo(f"\npoints = {points}")
        rospy.loginfo(f"\ncolors = {[observation.observation.observation_class for observation in all_observations]}")
        rospy.loginfo(f"\ncenters = {centers}")


node = MergeNode()
rospy.spin()
