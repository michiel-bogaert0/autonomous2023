#!/usr/bin/env python3

import numpy as np
from sklearn.neighbors import KDTree
from ugr_msgs.msg import (
    ObservationWithCovarianceStamped,
    ObservationWithCovarianceStampedArrayStamped,
)


class NaiveFusion:
    def __init__(self, euclidean_max_fusion_distance):
        self.euclidean_max_fusion_distance = euclidean_max_fusion_distance

    def fuse_observations(self, tf_sensor_msgs):
        """
        Fuse observations using a naive approach
        """
        results = ObservationWithCovarianceStampedArrayStamped()

        associations = self.kd_tree_merger(tf_sensor_msgs)
        fusion_observations = []
        for association in associations:
            if len(association) == 1:
                # Reshape covariance matrix from 3x3 to 1x9
                association[0].observation.covariance = tuple(
                    np.reshape(np.array(association[0].observation.covariance), (1, 9))[
                        0
                    ].tolist()
                )
                fusion_observations.append(association[0])
                continue
            fusion_observations.append(self.kalman_filter(association))
            # fusion_observations.append(self.euclidean_average(association))   # Use euclidean average instead of kalman filter
        results.observations = fusion_observations
        return results

    def kd_tree_merger(self, tf_sensor_msgs):
        """
        KDTree data association

        Distances between (transformed) observations are calculated using a KDTree. Detections of
        different sensors are then linked together based on their euclidean distance.
        Isolated cones are treated seperately and are not linked to any other cone.
        """

        # Create lists for all observations that have been coupled and the associations themselves
        associated_observations = []
        associations = []

        # Create list of sensors, observations & points of all incoming messages
        all_observations = []
        for msg in tf_sensor_msgs:
            all_observations.extend(msg.observations)

        if len(all_observations) < 1:
            return []
        all_points = list(
            map(
                lambda obs: [
                    obs.observation.observation.location.x,
                    obs.observation.observation.location.y,
                    obs.observation.observation.location.z,
                ],
                all_observations,
            )
        )

        # Create KDTree of all points
        kdtree_all = KDTree(all_points)

        # Create associations of observations
        for current_sensor_observations in tf_sensor_msgs:
            for current_obs in current_sensor_observations.observations:
                if current_obs in associated_observations:
                    continue

                association = [current_obs]

                # For each sensor other than current one, find best match for current_obs
                for sensor_msg in tf_sensor_msgs:
                    if current_obs in sensor_msg.observations:
                        continue

                    potential_matches = self.within_radius_unmatched(
                        all_observations=all_observations,
                        kdtree=kdtree_all,
                        root_obs=current_obs,
                        other_sensor_observations=sensor_msg.observations,
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
                            other_sensor_observations=current_sensor_observations.observations,
                        )[0]
                        == current_obs
                    ):
                        association.append(potential_matches[0])

                for obs in association:
                    associated_observations.append(obs)
                associations.append(association)
        return associations

    def within_radius_unmatched(
        self, all_observations, kdtree, root_obs, other_sensor_observations
    ):
        """
        Returns an ordered list of observations that are within max_fusion_distance radius of root_obs
        and of specified sensor type
        """

        # Find observations within max_fusion_distance radius
        indices, _ = kdtree.query_radius(
            [
                [
                    root_obs.observation.observation.location.x,
                    root_obs.observation.observation.location.y,
                    root_obs.observation.observation.location.z,
                ]
            ],
            r=self.euclidean_max_fusion_distance,
            return_distance=True,
            sort_results=True,
        )

        # Filter out observations other than those frome sensor and return an ordered list of potential matches
        return list(
            filter(
                lambda obs: obs in other_sensor_observations,
                [all_observations[i] for i in indices[0]],
            )
        )

    def kalman_filter(self, association):
        """
        Kalman filter

        Associated observations are merged using a Kalman filter
        to find an optimal estimate for cone location, type and covariance.
        Assumes len(association) > 1
        """

        # Initialize numpy arrays
        covariance_matrices = [
            np.reshape(np.array(obs.observation.covariance), (3, 3))
            for obs in association
        ]
        observation_locations = [
            np.reshape(
                np.array(
                    [
                        obs.observation.observation.location.x,
                        obs.observation.observation.location.y,
                        obs.observation.observation.location.z,
                    ]
                ),
                (3, 1),
            )
            for obs in association
        ]

        # Calculate new covariance matrix
        new_covariance = np.zeros((3, 3))
        inverse_covariances = [
            np.linalg.inv(covariance) for covariance in covariance_matrices
        ]
        for inverse in inverse_covariances:
            new_covariance += inverse
        new_covariance = np.linalg.inv(new_covariance)

        # Calculate weights of each observation
        weights = [
            np.matmul(new_covariance, inverse_covariance)
            for inverse_covariance in inverse_covariances
        ]

        # Calculate new observation location (weighted average)
        new_location = np.zeros((3, 1))
        for i in range(len(weights)):
            new_location += np.matmul(weights[i], observation_locations[i])

        # Create new observation
        fused_observation = ObservationWithCovarianceStamped()
        fused_observation.header = association[0].header
        fused_observation.header.frame_id = "kalman_fused"

        fused_observation.observation.observation.observation_class = association[
            0
        ].observation.observation.observation_class
        fused_observation.observation.observation.belief = association[
            0
        ].observation.observation.belief

        cam_in_obs = False
        for obs in association:
            if obs.header.frame_id == "camera":
                fused_observation.observation.observation.observation_class = (
                    obs.observation.observation.observation_class
                )
                fused_observation.observation.observation.belief = (
                    obs.observation.observation.belief
                )
                cam_in_obs = True
                break
        if not cam_in_obs:
            for obs in association:
                if obs.header.frame_id == "early_fusion":
                    fused_observation.observation.observation.observation_class = (
                        obs.observation.observation.observation_class
                    )
                    fused_observation.observation.observation.belief = (
                        obs.observation.observation.belief
                    )
                    break

        fused_observation.observation.covariance = tuple(
            np.reshape(new_covariance, (1, 9)).tolist()[0]
        )
        (
            fused_observation.observation.observation.location.x,
            fused_observation.observation.observation.location.y,
            fused_observation.observation.observation.location.z,
        ) = tuple((new_location[0][0], new_location[1][0], new_location[2][0]))
        return fused_observation

    def euclidean_average(self, association):
        average_observation = ObservationWithCovarianceStamped()
        average_observation.header = association[0].header
        average_observation.header.frame_id = "euclidean_fused"
        average_observation.observation.covariance = tuple(
            list(association[0].observation.covariance).copy()
        )

        (
            average_observation.observation.observation.location.x,
            average_observation.observation.observation.location.y,
            average_observation.observation.observation.location.z,
        ) = tuple(
            (
                sum([obs.observation.observation.location.x for obs in association])
                / len(association),
                sum([obs.observation.observation.location.y for obs in association])
                / len(association),
                sum([obs.observation.observation.location.z for obs in association])
                / len(association),
            )
        )

        return average_observation
