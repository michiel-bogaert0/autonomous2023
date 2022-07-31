import warnings

from .helper import Helpers
from .map import Map

import numpy as np


class FastMapping:
    """
    This class implements the 'FastMapping' algo
    """

    def __init__(
        self,
        threshold_distance=2,
        measurement_covariance=np.array([[0.05, 0], [0, 0.05]]),
    ):
        """
        One particle exists of a state (x, y, theta), a weight and a map with landmarks
        """

        # Parameters
        self.measurement_covariance = measurement_covariance  # =Q_t
        self.threshold_distance = threshold_distance

        self.particle_state = np.zeros(3)
        self.particle_map = Map(1)

    def get_map(self):
        """
        Returns the particle map
        """
        return self.particle_map

    def predict_observation(self, landmark_pos, pose):
        """
        Returns predicted (and expected) observation of the landmark_pos and H
        Args:
          - landmark_pos: the absolute landmark x, y position
          - pose: the pose of the car
        """
        theta = pose[2]
        dx, dy = landmark_pos - pose[:2]

        expected_observation = Helpers.landmark_pos_to_observation(landmark_pos, pose)
        expected_range, expected_bearing = expected_observation

        # This jacobian is only the part corresponding to the landmark because the kalman filter only changes landmark mu and sigma !
        # See course for more information
        # [[dr / dlx, dr / dly],
        #  [db / dlx, db / dly]]
        H = (
            np.array(
                [
                    [dx, dy],
                    [-1 * dy / expected_range, dx / expected_range],
                ]
            )
            / expected_range
        )

        return expected_observation, H

    def step(self, particle_state, observations):
        """
        This function applies the fastSLAM algorithm. It is no problem when no observations are given (so only control input and odometry)
        Args:
          u: the control inputs
          observations: a 2xN np array of N observations. An observation is [radius, bearing]
        """

        self.particle_state = particle_state

        # If no observations, just stop already
        if len(observations) == 0:
            return

        # Try to detect (based on the particles map) to which landmark these observations match
        expected_landmarks = [
            Helpers.observation_to_landmark_pos(observation[:2], self.particle_state)
            for observation in observations
        ]
        (
            observation_corresponding_distances,
            observation_corresponding_indices,
        ) = self.particle_map.search_corresponding_landmark_index(expected_landmarks)

        seen_indices = []

        for i, full_observation in enumerate(observations):

            observation = full_observation[:2]
            observation_class = full_observation[2]

            # Check if feature has already been seen before or not
            if (
                len(observation_corresponding_indices) == 0
                or observation_corresponding_distances[i] > self.threshold_distance
            ):

                # New landmark!
                landmark_mu = Helpers.observation_to_landmark_pos(
                    observation, self.particle_state
                )  # Initialize mu

                # Initialize EKF for this landmark and particle
                # The expected observation in the beginning is just the 'observation' itself, because we don't know the landmark...
                # Doesn't matter that much anyways

                # Get the "expected observation" (which here is just the observation itself...) and the jacobian of the observation measurement
                _, H = self.predict_observation(landmark_mu, self.particle_state)

                Hinv = H ** (-1)
                landmark_sigma = (
                    Hinv @ self.measurement_covariance
                ) @ Hinv.T  # Initialize sigma

                self.particle_map.add_landmark(
                    np.hstack((landmark_mu.reshape((2, 1)), landmark_sigma)),
                    observation_class,
                    score=1,
                )  # Add to map

            else:
                # Landmark already exists and is matched to one on this map
                matched_landmark_index = observation_corresponding_indices[i]

                seen_indices.append(matched_landmark_index)

                # Calculate the expected_observation and the corresponding jacobian
                expected_observation, H = self.predict_observation(
                    self.particle_map.get_landmark_mean(matched_landmark_index),
                    self.particle_state,
                )

                # Calculate Q (measurement covariance)
                landmark_covariance = self.particle_map.get_landmark_covariance(
                    matched_landmark_index
                )

                Q = ((H @ landmark_covariance) @ H.T) + self.measurement_covariance
                Qinv = np.linalg.inv(Q)

                # Calculate kalman gain
                K = (landmark_covariance @ H.T) @ Qinv

                # Update landmark mean and covariance
                observation_difference = observation - expected_observation

                # print(observation_difference, K)
                self.particle_map.set_landmark_mean(
                    matched_landmark_index,
                    self.particle_map.get_landmark_mean(matched_landmark_index)
                    + (K @ observation_difference),
                )

                self.particle_map.set_landmark_covariance(
                    matched_landmark_index,
                    (np.eye(2) - (K @ H)) @ landmark_covariance,
                )

                self.particle_map.increment_landmark_score(matched_landmark_index)
