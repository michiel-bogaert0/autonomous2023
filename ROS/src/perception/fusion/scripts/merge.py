#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped
)
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

        self.merge_distance_threshold = 2.5               # set threshold max fusion distance

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
        
        if second_observations is None:
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

        # Get covariance arrays of all observations
        early_covariances, late_covariances = [observation.covariance for observation in early_obs.observations], [observation.covariance for observation in late_obs.observations]

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
            
            
            time_transformed_early_obs: ObservationWithCovarianceArrayStamped = ROSNode.do_transform_observations(
                    early_obs, tf_early_to_base
            )
            time_transformed_late_obs: ObservationWithCovarianceArrayStamped = ROSNode.do_transform_observations(
                    late_obs, tf_late_to_base
            )

            for i in range(len(time_transformed_early_obs.observations)):
                time_transformed_early_obs.observations[i].covariance = early_covariances[i]
            for i in range(len(time_transformed_late_obs.observations)):
                time_transformed_late_obs.observations[i].covariance = late_covariances[i]
            
            self.kd_tree_merger(time_transformed_early_obs, time_transformed_late_obs)


        except Exception as e:
            rospy.logerr(
                f"Mergenode has caught an exception. Ignoring ObservationWithCovarianceArrayStamped messages... Exception: {e}"
            )

    def kd_tree_merger(self, observations1, observations2):
        rospy.loginfo(observations1)
        all_observations = observations1.observations + observations2.observations
        all_points = list(map(lambda obs: [obs.observation.location.x, obs.observation.location.y, obs.observation.location.z], all_observations))

        centers = []
        resulting_observations = []

        for observation in all_observations:
            if observation in observations1.observations:
                this_sensor_observations = observations1
                other_sensor_observations = observations2
            else:
                this_sensor_observations = observations2
                other_sensor_observations = observations1
            current_obs_set = [observation] + other_sensor_observations.observations
            sensor = this_sensor_observations.header.frame_id

            points = list(map(lambda obs: [obs.observation.location.x, obs.observation.location.y, obs.observation.location.z], current_obs_set))
            kdtree = KDTree(points)

            location = [observation.observation.location.x, observation.observation.location.y, observation.observation.location.z]
            distance, indices = kdtree.query([location], k=2)
            index = indices[0][1]

            distance2, indices2 = kdtree.query([points[index]], k=2)
            index2 = indices2[0][1]

            if points[index2] == location and distance[0][1] <= self.merge_distance_threshold:
                if sensor == self.lidar_sensor_name:
                    lidar_observation = observation
                    camera_observation = other_sensor_observations.observations[index-1]
                else:
                    camera_observation = observation
                    lidar_observation = other_sensor_observations.observations[index-1]

                center_observation = self.euclidean_average(lidar_observation, camera_observation)
                center_location = [center_observation.observation.location.x, center_observation.observation.location.y, center_observation.observation.location.z]
                if center_location not in centers:
                    centers.append(center_location)
                    resulting_observations.append(center_observation)

                rospy.loginfo(f"\nCenter found at: {center_location};\n\n")
                rospy.loginfo(f"\nLidar at: {lidar_observation.observation.location};\n\n")
            elif location not in centers:
                centers.append(location)
                resulting_observations.append(observation)
        
        rospy.loginfo(f"\npoints = {all_points}")
        rospy.loginfo(f"\ncolors = {[observation.observation.observation_class for observation in all_observations]}")
        rospy.loginfo(f"\ncenters = {centers}")

        result = ObservationWithCovarianceArrayStamped()
        result.header.seq = observations1.header.seq
        result.header.stamp.secs, result.header.stamp.nsecs = observations1.header.stamp.secs, observations1.header.stamp.nsecs
        result.header.frame_id = observations1.header.frame_id
        result.observations = resulting_observations
        self.publish(result)


    def euclidean_average(self, lidar_observation, camera_observation):
        average_observation = ObservationWithCovariance()
        average_observation.observation.observation_class = lidar_observation.observation.observation_class
        average_observation.observation.belief = lidar_observation.observation.belief
        average_observation.covariance = tuple(list(lidar_observation.covariance).copy())

        lidar_observation_location = [lidar_observation.observation.location.x, lidar_observation.observation.location.y, lidar_observation.observation.location.z]
        camera_observation_location = [camera_observation.observation.location.x, camera_observation.observation.location.y, camera_observation.observation.location.z]
        average_observation.observation.location.x, average_observation.observation.location.y, average_observation.observation.location.z = list((np.asarray(lidar_observation_location) + np.asarray(camera_observation_location))/2)

        return average_observation
    
    def kalman_filter(self, lidar_observation, camera_observation):
        return


node = MergeNode()
rospy.spin()
