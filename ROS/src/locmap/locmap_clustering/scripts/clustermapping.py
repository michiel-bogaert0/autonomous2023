#!/usr/bin/env python3

import numpy as np
from yaml import Mark
import rospy
import tf2_ros as tf
from clustering.clustering import Clustering
from geometry_msgs.msg import Point, TransformStamped
from node_fixture.node_fixture import AddSubscriber, ROSNode
from tf.transformations import euler_from_quaternion
from ugr_msgs.msg import Observation, Observations
from visualization_msgs.msg import MarkerArray, Marker


class ClusterMapping(ROSNode):
    def __init__(self) -> None:
        """
        The ClusterMapping algorithm provides LocMap with a way to create a map based on the clustering of observations.
        This is the ROS wrapper. For the implementation you have to go to src
        """

        super().__init__("clustermapping", False)

        # Parameters
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/car_odom")
        self.do_time_transform = rospy.get_param("~do_time_transform", True)
        self.observation_queue_size = rospy.get_param("~observation_queue_size", None)
        self.max_landmark_range = rospy.get_param("~max_landmark_range", 0)

        self.vis = rospy.get_param("~vis", True)

        self.eps = rospy.get_param("~clustering/eps", 0.5)
        self.min_sampling = rospy.get_param("~clustering/min_samples", 5)
        self.nr_of_classes = rospy.get_param("~clustering/nr_of_classes", 2)
        self.expected_nr_of_landmarks = rospy.get_param(
            "~clustering/expected_nr_of_landmarks", 1000
        )
        self.clustering_rate = rospy.get_param("~clustering/rate", 10)

        # Used to handle the sample output of the clusterer
        self.previous_sample_point = [0 for i in range(self.nr_of_classes)]

        # The clustering implementation
        self.clustering = Clustering(
            self.expected_nr_of_landmarks,
            self.nr_of_classes,
            self.eps,
            self.min_sampling,
        )

        # It is done this way instead of using decorators because we need to dynamically inject the queue size
        AddSubscriber("input/observations", self.observation_queue_size)(
            self.handle_observation_message
        )
        self.add_subscribers()

        # Helpers
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        rospy.loginfo(f"Clustering mapping node initialized!")

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

        all_landmarks = self.clustering.all_landmarks

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

        for clss, landmarks in enumerate(all_landmarks):

            for i, landmark in enumerate(landmarks):
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

                # The relative observations are range limited in both the distance (radius) and angle (relative to X-as)
                if self.max_landmark_range > 0:
                    distance = (landmark[0] - self.particle_state[0]) ** 2 + (
                        landmark[1] - self.particle_state[1]
                    ) ** 2
                    if distance < self.max_landmark_range ** 2:
                        observations.observations.append(new_obs)
                else:
                    observations.observations.append(new_obs)

                new_map_point = Observation()
                new_map_point.location = Point(x=landmark[0], y=landmark[1], z=0)
                new_map_point.observation_class = clss

                new_map.observations.append(new_map_point)

        self.publish("output/observations", observations)
        self.publish("output/map", new_map)

        # Visualisation
        #   Observations message are mapped to visualization_msgs/MarkerArray
        if self.vis:
            marker_array = MarkerArray()

            for i, obs in enumerate(observations.observations):
                marker = Marker()

                marker.header = observations.header

                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD;
                marker.id = i;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.pose.position.x = obs.location.x
                marker.pose.position.y = obs.location.y 

                marker.scale.x = 0.228; 
                marker.scale.y = 0.228;
                marker.scale.z = 0.325;

                marker.color.r = 0
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1

                marker.lifetime = rospy.Duration(0);

                marker_array.markers.append(marker)
            
            self.publish("/output/vis/observations", marker_array)

            marker_array = MarkerArray()

            for i, obs in enumerate(new_map.observations):
                marker = Marker()

                marker.header = observations.header

                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD;
                marker.id = i;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.pose.position.x = obs.location.x
                marker.pose.position.y = obs.location.y 

                marker.scale.x = 0.228; 
                marker.scale.y = 0.228;
                marker.scale.z = 0.325;

                marker.color.r = 0
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1

                marker.lifetime = rospy.Duration(0);

                marker_array.markers.append(marker)
            
            self.publish("/output/vis/map", marker_array)


        # Publish delta samples as well. These are the points used to cluster
        # Could be useful to estimate statistical distributions from
        # It is an analysis thingy, so only makes sense if relative to the world frame

        all_samples = self.clustering.samples
        samples = Observations()
        samples.header.frame_id = self.world_frame
        samples.header.stamp = rospy.Time().now()
        samples.observations = []

        for clss, landmarks in enumerate(all_samples):

            for i in range(
                self.previous_sample_point[clss], self.clustering.sizes[clss]
            ):

                landmark = landmarks[i]

                samples_point = Observation()
                samples_point.location = Point(x=landmark[0], y=landmark[1], z=0)
                samples_point.observation_class = clss

                samples.observations.append(samples_point)

            self.previous_sample_point[clss] = self.clustering.sizes[clss]

        self.publish("output/samples", samples)

        if self.vis:
            marker_array = MarkerArray()

            for i, obs in enumerate(samples.observations):
                marker = Marker()

                marker.header = observations.header

                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD;
                marker.id = i;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.pose.position.x = obs.location.x
                marker.pose.position.y = obs.location.y 

                marker.scale.x = 0.1; 
                marker.scale.y = 0.1;
                marker.scale.z = 0.02;

                marker.color.r = 1
                marker.color.g = 0
                marker.color.b = 0
                marker.color.a = 1

                marker.lifetime = rospy.Duration(0);

                marker_array.markers.append(marker)
            
            self.publish("/output/vis/samples", marker_array)



node = ClusterMapping()
node.start()
