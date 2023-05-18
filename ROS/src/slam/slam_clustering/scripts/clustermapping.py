#!/usr/bin/env python3

import numpy as np
import rospy
import tf2_ros as tf
from clustering.clustering import Clustering
from fs_msgs.msg import Cone
from geometry_msgs.msg import Point, TransformStamped
from node_fixture.node_fixture import AddSubscriber, ROSNode, DiagnosticArray, DiagnosticStatus, create_diagnostic_message
from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
    Particle,
    Particles,
)

from slam_clustering.srv import Reset, ResetRequest, ResetResponse


class ClusterMapping(ROSNode):
    def __init__(self) -> None:
        """
        The ClusterMapping algorithm provides slam with a way to create a map based on the clustering of observations.
        This is the ROS wrapper. For the implementation you have to go to src
        """

        super().__init__("clustermapping", False)

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # Parameters
        self.use_sim_time = rospy.get_param("use_sim_time", False)

        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/gt_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.do_time_transform = rospy.get_param("~do_time_transform", False)
        self.observation_queue_size = rospy.get_param("~observation_queue_size", 1)
        self.max_landmark_range = rospy.get_param("~max_landmark_range", 20) ** 2

        self.eps = rospy.get_param("~clustering/eps", 0.5)
        self.min_sampling = rospy.get_param("~clustering/min_samples", 5)
        self.mode = rospy.get_param("~clustering/mode", "global")
        self.expected_nr_of_landmarks = rospy.get_param(
            "~clustering/expected_nr_of_landmarks", 1000
        )
        self.clustering_rate = rospy.get_param("~clustering/rate", 10)

        # Used to handle the sample output of the clusterer
        self.previous_sample_point = 0

        self.started = False

        # The clustering implementation
        self.clustering = Clustering(
            self.mode,
            self.expected_nr_of_landmarks,
            self.eps,
            self.min_sampling,
        )
        self.particle_state = [0, 0, 0]

        # Add a service that allows us to reset the clustering when needed
        rospy.Service("clustermapping/reset", Reset, self.handle_reset_srv_request)

        # It is done this way instead of using decorators because we need to dynamically inject the queue size
        AddSubscriber("ugr/car/observations/lidar", self.observation_queue_size)(
            self.handle_observation_message
        )

        if self.use_sim_time:
            AddSubscriber("/clock", self.observation_queue_size)(
                self.detect_backwards_time_jump
            )

            self.current_clock = 0

        self.prev_t = rospy.Time().now().to_sec()
        self.add_subscribers()

        # Diagnostics Publisher
        self.diagnostics = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)

        # Helpers
        self.previous_clustering_time = rospy.Time.now().to_sec()
        self.initialized = True
        rospy.loginfo(f"Clustering mapping node initialized!")
        self.diagnostics.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.OK,
                        name="[SLAM CM] Status",
                        message="Initialized.",
                    )
                )

    def handle_reset_srv_request(self, req: ResetRequest):
        """
        This is the service handler that handles a cluster reset.
        Resets the cluster upon receiving this request

        Args:
            req: the ResetRequest object (empty)

        Returns:
            ResetResponse (empty)
        """
        rospy.logwarn("Received reset request")

        self.reset()
        self.diagnostics.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.OK,
                        name="[SLAM CM] Reset",
                        message="FastMapping reset.",
                    )
                )

        return ResetResponse()

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
            self.diagnostics.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.WARN,
                        name="[SLAM CM] Backwards time",
                        message="FastMapping reset.",
                    )
                )

            self.reset()

    def reset(self):
        """
        Resets this node to initial conditions.
        Basically clears the cluster samples, clears the tf buffer and sets everything back to starting conditions
        """
        self.clustering.reset()

        self.current_clock = 0
        self.previous_sample_point = 0
        self.started = False
        self.cleared_vis = 0

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        self.previous_clustering_time = rospy.Time.now().to_sec()

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

        if not self.initialized:
            rospy.logwarn(
                "Node is still initializing. Dropping incoming ObservationWithCovarianceArrayStamped message..."
            )
            return

        try:
            # Transform the observations!
            # This only transforms from sensor frame to base link frame, which should be a static transformation in normal conditions
            transformed_observations: ObservationWithCovarianceArrayStamped = (
                ROSNode.do_transform_observations(
                    observations,
                    self.tf_buffer.lookup_transform(
                        observations.header.frame_id.strip("/"),
                        self.base_link_frame,
                        rospy.Time(0),
                    ),
                )
            )

            if self.do_time_transform:
                # Now do a "time transformation" to keep delays in mind
                transform: TransformStamped = self.tf_buffer.lookup_transform_full(
                    self.base_link_frame,
                    rospy.Time(),
                    self.base_link_frame,
                    observations.header.stamp,
                    self.world_frame,  # Needs a fixed frame to use as fixture for the transformation
                    rospy.Duration(0),
                )
                time_transformed_observations = ROSNode.do_transform_observations(
                    transformed_observations, transform
                )

                self.process_observations(time_transformed_observations)
            else:
                self.process_observations(transformed_observations)
        except Exception as e:
            rospy.logerr(
                f"ClusterMapping has caught an exception. Ignoring ObservationWithCovarianceArrayStamped message... Exception: {e}"
            )

    def process_observations(self, observations: ObservationWithCovarianceArrayStamped):
        """
        This function gets called when an ObservationWithCovarianceArrayStamped message needs to be processed into the clustering

        Args:
            - observations: The ObservationWithCovarianceArrayStamped message that needs to be processed
        """

        # Look up the base_link frame relative to the world at the time of the observations.
        # This is needed to correctly transform the observations on the "map" (=world frame) before clustering
        transform: TransformStamped = self.tf_buffer.lookup_transform(
            self.world_frame, self.base_link_frame, observations.header.stamp
        )

        if transform.transform.translation.x < 1 and not self.started:
            return
        else:
            self.started = True


        _, _, yaw = euler_from_quaternion(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        )

        if abs((self.particle_state[2] - yaw) / (rospy.Time().now().to_sec() - self.prev_t)) > 0.1:
            self.particle_state = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw,
            ]
            self.prev_t = rospy.Time().now().to_sec()
            return
        

        self.prev_t = rospy.Time().now().to_sec()
        self.particle_state = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        ]

        world_observations: ObservationWithCovarianceArrayStamped = (
            ROSNode.do_transform_observations(observations, transform)
        )

        for observation in world_observations.observations:

            distance = (observation.observation.location.x - self.particle_state[0]) ** 2 + (
                observation.observation.location.y - self.particle_state[1]
            ) ** 2


            if distance <= self.max_landmark_range and distance > 0.05:

                self.clustering.add_sample(
                    np.array([observation.observation.location.x, observation.observation.location.y]),
                    int(observation.observation.observation_class),
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
        # Note how the output is also an ObservationWithCovarianceArrayStamped message!
        observations = ObservationWithCovarianceArrayStamped()
        observations.header.frame_id = self.base_link_frame
        observations.header.stamp = rospy.Time().now()
        observations.observations = []

        # Also publish the same result but now relative to world_frame (so the "map" estimate)
        new_map = ObservationWithCovarianceArrayStamped()
        new_map.header.frame_id = self.world_frame
        new_map.header.stamp = rospy.Time().now()
        new_map.observations = []

        for j in range(count):
            clss = classes[j]
            landmark = landmarks[j]

            new_obs = ObservationWithCovariance()
            new_obs.observation.location = Point(
                x=landmark[0] - self.particle_state[0],
                y=landmark[1] - self.particle_state[1],
                z=0,
            )

            # Apply rotation
            x = (
                np.cos(-self.particle_state[2]) * new_obs.observation.location.x
                - np.sin(-self.particle_state[2]) * new_obs.observation.location.y
            )
            y = (
                np.sin(-self.particle_state[2]) * new_obs.observation.location.x
                + np.cos(-self.particle_state[2]) * new_obs.observation.location.y
            )

            distance = (landmark[0] ** 2 + landmark[1] ** 2) ** (1 / 2)

            if distance > 0.1:

                new_obs.observation.location.x = x
                new_obs.observation.location.y = y

                new_obs.observation.observation_class = clss

                observations.observations.append(new_obs)

                new_map_point = ObservationWithCovariance()
                new_map_point.observation.location = Point(x=landmark[0], y=landmark[1], z=0)
                new_map_point.observation.observation_class = clss

                new_map.observations.append(new_map_point)

        self.publish("output/observations", observations)
        self.publish("output/map", new_map)

        # Publish delta samples as well. These are the points used to cluster
        # Could be useful to estimate statistical distributions from
        # It is an analysis thingy, so only makes sense if relative to the world frame

        samples = ObservationWithCovarianceArrayStamped()
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
                samples_point = ObservationWithCovariance()
                samples_point.observation.location = Point(x=sample[0], y=sample[1], z=0)
                samples_point.observation.observation_class = int(clss)

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
            samples_point = ObservationWithCovariance()
            samples_point.observation.location = Point(x=sample[0], y=sample[1], z=0)
            samples_point.observation.observation_class = int(clss)

            samples.observations.append(samples_point)

            # Now as a particle
            part = Particle()
            part.weight = 0
            part.position = Point(x=sample[0], y=sample[1], z=0)

            samples_as_particles.particles.append(part)

        self.previous_sample_point = self.clustering.size

        self.publish("output/samples", samples)

node = ClusterMapping()
node.start()
