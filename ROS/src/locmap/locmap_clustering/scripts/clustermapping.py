#!/usr/bin/env python3

import numpy as np
import rospy
import tf2_ros as tf
from clustering.clustering import Clustering
from fs_msgs.msg import Cone
from geometry_msgs.msg import Point, TransformStamped
from locmap_vis import LocMapVis
from node_fixture.node_fixture import AddSubscriber, ROSNode
from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion
from ugr_msgs.msg import Observation, Observations, Particle, Particles


class ClusterMapping(ROSNode):
    def __init__(self) -> None:
        """
        The ClusterMapping algorithm provides LocMap with a way to create a map based on the clustering of observations.
        This is the ROS wrapper. For the implementation you have to go to src
        """

        super().__init__("clustermapping", False)

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # Parameters
        self.use_sim_time = rospy.get_param("use_sim_time", False)

        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/car_odom")
        self.do_time_transform = rospy.get_param("~do_time_transform", True)
        self.observation_queue_size = rospy.get_param("~observation_queue_size", None)
        self.max_landmark_range = rospy.get_param("~max_landmark_range", 0) ** 2

        self.vis = rospy.get_param("~vis", True)
        self.blue_cone_model_url = rospy.get_param(
            "~vis/cone_models/blue",
            "https://storage.googleapis.com/learnmakeshare_cdn_public/blue_cone_final.dae",
        )
        self.yellow_cone_model_url = rospy.get_param(
            "~vis/cone_models/yellow",
            "https://storage.googleapis.com/learnmakeshare_cdn_public/yellow_cone_final.dae",
        )
        self.vis_namespace = rospy.get_param("~vis/namespace", "locmap_vis")
        self.vis_lifetime = rospy.get_param("~vis/lifetime", 3)
        self.vis_sample_color = rospy.get_param("~vis/sample_color", "g")
        self.vis_handler = LocMapVis(
            [self.blue_cone_model_url, self.yellow_cone_model_url]
        )

        self.eps = rospy.get_param("~clustering/eps", 0.5)
        self.min_sampling = rospy.get_param("~clustering/min_samples", 5)
        self.mode = rospy.get_param("~clustering/mode", "local")
        self.expected_nr_of_landmarks = rospy.get_param(
            "~clustering/expected_nr_of_landmarks", 1000
        )
        self.clustering_rate = rospy.get_param("~clustering/rate", 10)

        # Used to handle the sample output of the clusterer
        self.previous_sample_point = 0

        # The clustering implementation
        self.clustering = Clustering(
            self.mode,
            self.expected_nr_of_landmarks,
            self.eps,
            self.min_sampling,
        )

        # It is done this way instead of using decorators because we need to dynamically inject the queue size
        AddSubscriber("input/observations", self.observation_queue_size)(
            self.handle_observation_message
        )

        if self.use_sim_time:
            AddSubscriber("/clock", self.observation_queue_size)(
                self.detect_backwards_time_jump
            )

            self.current_clock = 0

        self.add_subscribers()

        # Helpers
        self.previous_clustering_time = rospy.Time.now().to_sec()
        self.cleared_vis = 0

        self.initialized = True
        rospy.loginfo(f"Clustering mapping node initialized!")

    def detect_backwards_time_jump(self, _, clock: Clock):
        """
        Handler that detects when the published clock time (on /clock) has jumped back in time
        When a jump back has been detected, the state of the node gets reset.

        Only works if the rosparam 'use_sim_time' is set to True and there is a (simulated) clock source, like a rosbag

        Args:
            clock: the Clock message, coming from Clock
        """

        if self.current_clock == 0:
            self.current_clock = clock.clock.to_sec()
            return

        if self.current_clock > clock.clock.to_sec():
            rospy.logwarn(
                f"A backwards jump in time of {self.current_clock - clock.clock.to_sec()} has been detected. Resetting the clusterer..."
            )

            self.clustering.reset()

            self.current_clock = 0
            self.previous_sample_point = 0
            self.cleared_vis = 0

            self.tf_buffer = tf.Buffer()
            self.tf_listener = tf.TransformListener(self.tf_buffer)

            self.previous_clustering_time = rospy.Time.now().to_sec()

    # See the constructor for the subscriber registration.
    # The '_' is just because it doesn't use a decorator, so it injects 'self' twice
    def handle_observation_message(self, _, observations: Observations):
        """
        Handles the incoming observations.
        These observations must be (time) transformable to the base_link frame provided

        Args:
            - observations: The observations to process
        """

        if not self.initialized:
            rospy.logwarn(
                "Node is still initializing. Dropping incoming Observations message..."
            )
            return

        try:
            # Transform the observations!
            # This only transforms from sensor frame to base link frame, which should be a static transformation in normal conditions
            transformed_observations: Observations = ROSNode.do_transform_observations(
                observations,
                self.tf_buffer.lookup_transform(
                    observations.header.frame_id.strip("/"),
                    self.base_link_frame,
                    rospy.Time(0),
                ),
            )

            if self.do_time_transform:
                # Now do a "time transformation" to keep delays in mind
                transform: TransformStamped = self.tf_buffer.lookup_transform_full(
                    self.base_link_frame,
                    rospy.Time(),
                    self.base_link_frame,
                    observations.header.stamp,
                    self.world_frame,  # Needs a fixed frame to use as fixture for the transformation
                    rospy.Duration(1),
                )
                time_transformed_observations = ROSNode.do_transform_observations(
                    transformed_observations, transform
                )

                self.process_observations(time_transformed_observations)
            else:
                self.process_observations(transformed_observations)
        except Exception as e:
            rospy.logerr(
                f"ClusterMapping has caught an exception. Ignoring Observations message... Exception: {e}"
            )

    def process_observations(self, observations: Observations):
        """
        This function gets called when an Observations message needs to be processed into the clustering

        Args:
            - observations: The Observations message that needs to be processed
        """

        # Look up the base_link frame relative to the world at the time of the observations.
        # This is needed to correctly transform the observations on the "map" (=world frame) before clustering
        transform: TransformStamped = self.tf_buffer.lookup_transform(
            self.world_frame, self.base_link_frame, observations.header.stamp
        )

        _, _, yaw = euler_from_quaternion(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        )

        self.particle_state = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        ]

        world_observations: Observations = ROSNode.do_transform_observations(
            observations, transform
        )

        for observation in world_observations.observations:

            distance = (observation.location.x - self.particle_state[0]) ** 2 + (
                observation.location.y - self.particle_state[1]
            ) ** 2

            if distance <= self.max_landmark_range and distance > 0.1:

                self.clustering.add_sample(
                    np.array([observation.location.x, observation.location.y]),
                    int(observation.observation_class),
                )

        # Clustering itself is rate limited to limit performance impact
        if (
            self.previous_clustering_time
            < rospy.Time.now().to_sec() - 1 / self.clustering_rate
        ):
            self.previous_clustering_time = rospy.Time.now().to_sec()
            self.clustering.cluster()

        count, classes, landmarks = self.clustering.all_landmarks

        # Publish all the resulting points as "observations" relative to base_link
        # Note how the output is also an Observations message!
        observations = Observations()
        observations.header.frame_id = self.base_link_frame
        observations.header.stamp = rospy.Time().now()
        observations.observations = []

        # Also publish the same result but now relative to world_frame (so the "map" estimate)
        new_map = Observations()
        new_map.header.frame_id = self.world_frame
        new_map.header.stamp = rospy.Time().now()
        new_map.observations = []

        for j in range(count):
            clss = classes[j]
            landmark = landmarks[j]

            new_obs = Observation()
            new_obs.location = Point(
                x=landmark[0] - self.particle_state[0],
                y=landmark[1] - self.particle_state[1],
                z=0,
            )

            # Apply rotation
            x = (
                np.cos(-self.particle_state[2]) * new_obs.location.x
                - np.sin(-self.particle_state[2]) * new_obs.location.y
            )
            y = (
                np.sin(-self.particle_state[2]) * new_obs.location.x
                + np.cos(-self.particle_state[2]) * new_obs.location.y
            )

            new_obs.location.x = x
            new_obs.location.y = y

            new_obs.observation_class = clss

            observations.observations.append(new_obs)

            new_map_point = Observation()
            new_map_point.location = Point(x=landmark[0], y=landmark[1], z=0)
            new_map_point.observation_class = clss

            new_map.observations.append(new_map_point)

        self.publish("output/observations", observations)
        self.publish("output/map", new_map)

        # Publish delta samples as well. These are the points used to cluster
        # Could be useful to estimate statistical distributions from
        # It is an analysis thingy, so only makes sense if relative to the world frame

        samples = Observations()
        samples.header.frame_id = self.world_frame
        samples.header.stamp = rospy.Time().now()
        samples.observations = []

        samples_as_particles = Particles()
        samples_as_particles.header = samples.header

        # If applicable, first take the samples until the end of the array
        if self.mode == "local" and self.clustering.size < self.previous_sample_point:
            for clss, sample in zip(
                self.clustering.sample_classes[self.previous_sample_point :],
                self.clustering.samples[self.previous_sample_point :],
            ):
                samples_point = Observation()
                samples_point.location = Point(x=sample[0], y=sample[1], z=0)
                samples_point.observation_class = clss

                samples.observations.append(samples_point)

                # Now as a particle
                part = Particle()
                part.weight = 0
                part.position = Point(x=sample[0], y=sample[1], z=0)

            self.previous_sample_point = 0

        # Now do the normal thing, up until size
        for clss, sample in zip(
            self.clustering.sample_classes[
                self.previous_sample_point : self.clustering.size
            ],
            self.clustering.samples[self.previous_sample_point : self.clustering.size],
        ):
            samples_point = Observation()
            samples_point.location = Point(x=sample[0], y=sample[1], z=0)
            samples_point.observation_class = clss

            samples.observations.append(samples_point)

            # Now as a particle
            part = Particle()
            part.weight = 0
            part.position = Point(x=sample[0], y=sample[1], z=0)

            samples_as_particles.particles.append(part)

        self.previous_sample_point = self.clustering.size

        self.publish("output/samples", samples)

        if self.vis:

            if self.cleared_vis < 5:

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
                    self.vis_handler.delete_markerarray(
                        self.vis_namespace + "/samples"
                    ),
                )

                self.cleared_vis += 1

            marker_array = self.vis_handler.observations_to_markerarray(
                observations, self.vis_namespace + "/observations", 0, False
            )
            self.publish("/output/vis", marker_array)

            marker_array = self.vis_handler.observations_to_markerarray(
                new_map, self.vis_namespace + "/map", 0, False
            )
            self.publish("/output/vis", marker_array)

            marker_array = self.vis_handler.particles_to_markerarray(
                Particles(
                    header=samples_as_particles.header,
                    particles=[
                        particle
                        for i, particle in enumerate(samples_as_particles.particles)
                        if samples.observations[i].observation_class == Cone.BLUE
                    ],
                ),
                self.vis_namespace + "/samples",
                self.vis_lifetime,
                "b",
                True,
            )
            self.publish("/output/vis", marker_array)

            marker_array = self.vis_handler.particles_to_markerarray(
                Particles(
                    header=samples_as_particles.header,
                    particles=[
                        particle
                        for i, particle in enumerate(samples_as_particles.particles)
                        if samples.observations[i].observation_class == Cone.YELLOW
                    ],
                ),
                self.vis_namespace + "/samples",
                self.vis_lifetime,
                "y",
                True,
            )
            self.publish("/output/vis", marker_array)


node = ClusterMapping()
node.start()
