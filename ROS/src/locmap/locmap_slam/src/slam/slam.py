#!/usr/bin/env python3

from abc import ABC, abstractmethod

import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import Point, Pose, PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import AddSubscriber, ROSNode
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
    Particle,
    Particles,
)

from locmap_vis import LocMapVis

"""
Generic SLAM node should have the following behaviour

Input:
    - ObservationWithCovarianceArrayStamped (with a header that should correspond to the frame in which the observations where taken). Must be transformable to base_link
    - Odometry used as input for the SLAM input. Gets transformed to the world_frame

Outputs:
    - ObservationWithCovarianceArrayStamped (relative to base_link frame)
    - Map estimate (relative to the world frame)
"""


class SLAMNode(ROSNode, ABC):
    """
    This node is the generic interface for a SLAM module/node.
    It takes in odometry and observations and spits out a map estimate
    (which is modelled as an ObservationWithCovarianceArrayStamped message to support daisy chaining of SLAM nodes),
    + some localization feedback in the future
    """

    def __init__(self, name):
        super().__init__(name, False)
        """
        Args:
            - name: The name of the node

        A note about the general rospy parameters:
            - base_link_frame: the base_link frame of the robot
            - world_frame: the frame that is deemed "fixed" in the context of the node.
                            So this should be odom or map, depending on if you are working globally or locally.
                            This frame is also
                            This is also the frame in which the resulting map estimate is published
        """
        # Parameters
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/car_odom")
        self.do_time_transform = rospy.get_param("~do_time_transform", True)
        self.observation_queue_size = rospy.get_param("~observation_queue_size", None)
        self.max_landmark_range = rospy.get_param("~max_landmark_range", 0)

        self.vis = rospy.get_param("~vis", True)
        self.vis_namespace = rospy.get_param("~vis/namespace", "locmap_vis")
        self.vis_lifetime = rospy.get_param("~vis/lifetime", 3)
        self.vis_handler = LocMapVis()

        AddSubscriber("input/observations", self.observation_queue_size)(
            self.handle_observation_message
        )

        self.add_subscribers()

        # Helpers
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # Clear visualisation
        if self.vis:
            self.publish(
                "/output/vis",
                self.vis_handler.delete_markerarray(
                    self.vis_namespace + "/observations"
                ),
            )
            self.publish(
                "/output/vis",
                self.vis_handler.delete_markerarray(self.vis_namespace + "/map"),
            )
            self.publish(
                "/output/vis",
                self.vis_handler.delete_markerarray(self.vis_namespace + "/particles"),
            )

        rospy.loginfo(f"SLAM node ({name}) initialized!")

    # See the constructor for the subscriber registration.
    # The '_' is just because it doesn't use a decorator, so it injects 'self' twice
    def handle_observation_message(
        self, _, observations: ObservationWithCovarianceArrayStamped
    ):
        """
        Handles the incoming observations.
        These observations must be (time) transformable to the base_link frame provided

        Args:
            - observations: The observations to process
        """

        # Transform the observations!
        # This only transforms from sensor frame to base link frame, which should be a static transformation in normal conditions
        transformed_observations: ObservationWithCovarianceArrayStamped = (
            ROSNode.do_transform_observations(
                observations,
                self.tf_buffer.lookup_transform(
                    observations.header.frame_id.strip("/"),
                    self.base_link_frame,
                    rospy.Time(0),
                    rospy.Duration(0.1),
                ),
            )
        )

        # Now do a "time transformation" to keep delays in mind
        transform: TransformStamped = self.tf_buffer.lookup_transform_full(
            self.base_link_frame,
            rospy.Time.now(),
            self.base_link_frame,
            observations.header.stamp,
            self.world_frame,  # Needs a fixed frame to use as fixture for the transformation
            rospy.Duration(1),
        )

        if self.do_time_transform:
            time_transformed_observations = ROSNode.do_transform_observations(
                transformed_observations, transform
            )

            self.process_observations(time_transformed_observations)
        else:
            self.process_observations(transformed_observations)

    @AddSubscriber("input/odometry")
    def handle_odometry_message(self, odometry: Odometry):
        """
        Handles incoming Odometry messages. The header and frames should of course match with what is expected

        Args:
            - odometry: Odometry message
        """

        if odometry.child_frame_id != self.base_link_frame:
            rospy.logerr(
                f"Received an odometry message with an invalid child_frame_id (Received '{odometry.child_frame_id}' but expected '{self.base_link_frame}')"
            )
            return

        if odometry.header.frame_id != self.world_frame:
            rospy.logerr(
                f"Received an odometry message with an invalid frame_id (Received '{odometry.header.frame_id}' but expected '{self.world_frame}')"
            )
            return

        self.process_odometry(odometry)

    # These methods should be implemented by the algos themselves
    @abstractmethod
    def process_odometry(self, odometry: Odometry):
        pass

    @abstractmethod
    def process_observations(self, observations: ObservationWithCovarianceArrayStamped):
        pass

    @abstractmethod
    def get_predictions(self):
        pass

    def publish_result(self, timestamp=None):
        """
        Publishes the resulting esimtates

        Args:
            timestamp?: the timestamp to publish at. Leave empty to use current time
        """

        timestamp = timestamp if timestamp else rospy.Time.now()

        # Get the map and metadata corresponding to it
        (
            state_prediction,
            map_prediction,
            landmark_classes,
            path_prediction,
            particle_states,
            particle_weigthts,
        ) = self.get_predictions()

        # Publish the "observations" relative to base_link
        observations = ObservationWithCovarianceArrayStamped()
        observations.header.frame_id = self.base_link_frame
        observations.header.stamp = timestamp
        observations.observations = []

        # Also publish the same observations but now relative to world_frame (so the "map" estimate)
        new_map = ObservationWithCovarianceArrayStamped()
        new_map.header.frame_id = self.world_frame
        new_map.header.stamp = timestamp
        new_map.observations = []
        for i, obs_location in enumerate(map_prediction[:, 0::3].T):
            new_obs = ObservationWithCovariance()
            new_obs.observation.location = Point(
                x=obs_location[0] - state_prediction[0],
                y=obs_location[1] - state_prediction[1],
                z=0,
            )

            # Apply rotation
            x = (
                np.cos(-state_prediction[2]) * new_obs.observation.location.x
                - np.sin(-state_prediction[2]) * new_obs.observation.location.y
            )
            y = (
                np.sin(-state_prediction[2]) * new_obs.observation.location.x
                + np.cos(-state_prediction[2]) * new_obs.observation.location.y
            )

            new_obs.observation.location.x = x
            new_obs.observation.location.y = y

            new_obs.observation.observation_class = landmark_classes[i]

            if self.max_landmark_range > 0:
                distance = (obs_location[0] - state_prediction[0]) ** 2 + (
                    obs_location[1] - state_prediction[1]
                ) ** 2
                if distance < self.max_landmark_range**2:
                    observations.observations.append(new_obs)
            else:
                observations.observations.append(new_obs)

            new_map_point = ObservationWithCovariance()
            new_map_point.observation.location = Point(x=obs_location[0], y=obs_location[1], z=0)
            new_map_point.observation.observation_class = landmark_classes[i]

            new_map.observations.append(new_map_point)

        self.publish("output/observations", observations)
        self.publish("output/map", new_map)

        # Visualisation
        if self.vis:
            marker_array = self.vis_handler.observations_to_markerarray(
                observations, self.vis_namespace + "/observations", 0, False
            )
            self.publish("/output/vis", marker_array)

            marker_array = self.vis_handler.observations_to_markerarray(
                new_map, self.vis_namespace + "/map", 0, False
            )
            self.publish("/output/vis", marker_array)

        # Publish slam state
        slam_prediction = Odometry()
        slam_prediction.pose.pose.position.x = state_prediction[0]
        slam_prediction.pose.pose.position.y = state_prediction[1]
        slam_prediction.pose.pose.orientation.z = 1
        slam_prediction.pose.pose.orientation.w = state_prediction[2]

        slam_prediction.header.stamp = timestamp
        slam_prediction.header.frame_id = self.world_frame
        slam_prediction.child_frame_id = self.base_link_frame

        self.publish("output/odom", slam_prediction)

        # SLAM path prediction
        path = PoseArray()
        path.header.stamp = timestamp
        path.header.frame_id = self.world_frame

        for pos in path_prediction:
            pose = Pose()
            pose.position = Point(pos[0], pos[1], 0)
            path.poses.append(pose)

        self.publish("output/path", path)

        # SLAM particles
        particle_set = Particles()
        particle_set.header.stamp = timestamp
        particle_set.header.frame_id = self.world_frame

        for particle, weight in zip(particle_states, particle_weigthts):
            particle_set.particles.append(
                Particle(position=Point(particle[0], particle[1], 0), weight=weight)
            )

        marker_array = self.vis_handler.particles_to_markerarray(
            particle_set, self.vis_namespace + "/particles", 0, "r", False
        )
        self.publish("output/vis/particles", marker_array)

        self.publish("output/particles", particle_set)
