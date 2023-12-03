#!/usr/bin/env python3

# import numpy as np
# import rospy
from sklearn.neighbors import KDTree
from ugr_msgs.msg import (  # ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class NaiveFusion:
    def __init__(self, euclidean_max_fusion_distance):
        self.euclidean_max_fusion_distance = euclidean_max_fusion_distance
        return

    def fuse_observations(self, tf_sensor_msgs):
        """
        Fuse observations using a naive approach
        """
        results = ObservationWithCovarianceArrayStamped()

        associations = self.kd_tree_merger(tf_sensor_msgs)
        fusion_observations = []
        for association in associations:
            if len(association) == 1:
                fusion_observations.append(association[0])
                continue
            fusion_observations.append(self.kalman_filter(association))

        return results

    def kd_tree_merger(self, tf_sensor_msgs):
        """
        KDTree data association

        Distances between (transformed) observations are calculated using a KDTree. Detections of
        different sensors are then linked together based on their euclidean distance.
        Isolated cones are treated seperately and are not linked to any other cone.
        """

        # Create lists for coupled observations, actual associations and isolated observations
        associated_observations = []
        associations = []

        # Create list of sensors, observations & points of all incoming messages
        sensors = [msg.header.frame_id for msg in tf_sensor_msgs]
        all_observations = []
        for msg in tf_sensor_msgs:
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

        # Create KDTree of all points
        kdtree_all = KDTree(all_points)

        # Create associations of observations
        for msg in tf_sensor_msgs:
            for current_obs in msg.observations:
                if current_obs in associated_observations:
                    continue

                association = [current_obs]

                # For each sensor, find all observations within radius of current_obs
                for sensor in sensors:
                    if sensor == current_obs.frame_id:
                        continue

                    potential_matches = self.within_radius_unmatched(
                        all_observations=all_observations,
                        kdtree=kdtree_all,
                        root_obs=current_obs,
                        sensor=sensor,
                    )

                    # If no matches of sensor type are found within max radius, continue to next sensor
                    if len(potential_matches) == 0:
                        continue

                    # If closest match is already associated, continue to next sensor
                    if potential_matches[0] in associated_observations:
                        continue

                    # Check if closest match for current_obs also has current_obs as closest match
                    if (
                        self.within_radius_unmatched(
                            all_observations=all_observations,
                            kdtree=kdtree_all,
                            root_obs=potential_matches[0],
                            sensor=current_obs.frame_id,
                        )[0]
                        == current_obs
                    ):
                        association.append(potential_matches[0])

                for obs in association:
                    associated_observations.append(obs)
                associations.append(association)

        return associations

    def within_radius_unmatched(self, all_observations, kdtree, root_obs, sensor):
        """
        Returns an ordered list of observations that are within max_fusion_distance radius of root_obs
        and of specified sensor type
        """

        # Find observations within max_fusion_distance radius
        _, indices = kdtree.query_radius(
            [
                [
                    root_obs.observation.location.x,
                    root_obs.observation.location.y,
                    root_obs.observation.location.z,
                ]
            ],
            r=self.euclidean_max_fusion_distance,
            return_distance=True,
            sort_results=True,
        )

        # Filter out observations other than those frome sensor and return an ordered list of potential matches
        return list(
            filter(
                lambda obs: obs.frame_id == sensor,
                [all_observations[i] for i in indices[0]],
            )
        )

    def kalman_filter(self):
        """
        Kalman filter

        Associated observations are merged using a Kalman filter
        to find an optimal estimate for cone location, type and covariance.
        """
        return
