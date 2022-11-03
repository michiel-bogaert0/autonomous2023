import numpy as np

"""
Class with general helper functions for FastSLAM
"""


class Helpers:
    @staticmethod
    def calculate_observation_jacobian(observation, pose):
        """
        Returns measurement jacobian
        """
        landmark_pos = Helpers.observation_to_landmark_pos(observation, pose)
        theta = pose[2]
        dx, dy = landmark_pos - pose[:2]

        expected_range, expected_bearing = observation

        # This jacobian is only the part corresponding to the landmark because the kalman filter only changes landmark mu and sigma !
        # One jacobian has dimensions of 2x2
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

        return H

    @staticmethod
    def calculate_observation_jacobians(observations, pose):
        """
        Same as calculate_observation_jacobian but for multiple observations in parallel
        """
        landmark_poses = Helpers.observations_to_landmark_poses(observations, pose)

        diff = landmark_poses - pose[:2]  # N x 2 amtrix [[dx, dy], [dx, dy], ...]

        # The jacobian is one giant 2N x 2 array
        # Take a look at the single function for more details about the jacobian. Here it is all about performance.
        H = np.zeros((2 * len(observations), 2))

        ranges = np.array([observations[:, 0], observations[:, 0]]).T.copy()

        H[0::2] = diff / ranges
        H[1::2] = diff[:, [1, 0]] / ranges**2
        H[1::2, 0] *= -1

        return H

    @staticmethod
    def observation_to_landmark_pos(observation, pose):
        """
        Method to transform a single observation [range, bearing] to a position [x, y]

        Args:
            observation: [range, bearing] numpy array
            pose: [x, y, theta] numpy array that represents the pose to use as reference

        Returns: [x_landmark, y_landmark] numpy array
        """
        radius, bearing = observation
        x, y, theta = pose

        return np.array(
            [x + radius * np.cos(bearing + theta), y + radius * np.sin(bearing + theta)]
        )

    @staticmethod
    def observations_to_landmark_poses(observations, pose):
        """
        The multi-version of observation_to_landmark_pos

        Args:
          observation: [[range0, bearing0], [range1, bearing1], ...] Nx2 np array of [range, bearings]
          pose: [x, y, theta] numpy array that represents the pose to use as reference
        """

        landmarks_poses = np.zeros(observations.shape)

        ranges = observations[:, 0]
        bearings = observations[:, 1]

        landmarks_poses[:, 0] = pose[0] + ranges * np.cos(bearings + pose[2])
        landmarks_poses[:, 1] = pose[1] + ranges * np.sin(bearings + pose[2])

        return landmarks_poses

    @staticmethod
    def landmark_pos_to_observation(landmark_pos, pose):
        """
        Method to transform a single landmark position [x, y] to an observation [range, bearing]
        """
        theta = pose[2]
        dx, dy = landmark_pos - pose[:2]

        return np.array(
            [np.sqrt(dx**2 + dy**2), Helpers.pi_2_pi(np.arctan2(dy, dx) - theta)]
        )

    @staticmethod
    def landmark_poses_to_observations(landmark_poses, pose):
        """
        The multi-version of landmark_pos_to_observation

        Args:
          landmark_poses: [[x0, y0], [x1, y1], ...] Nx2 np array of [x, y] landmarks
          pose: [x, y, theta] numpy array that represents the pose to use as reference
        """

        diff = landmark_poses - pose[:2]
        ranges = np.sqrt(np.sum(diff**2, axis=1))

        bearings = Helpers.pi_2_pi(np.arctan2(diff[:, 1], diff[:, 0]) - pose[2])

        temp = Helpers.landmark_pos_to_observation(landmark_poses[0], pose)

        return np.vstack((ranges, bearings)).T

    @staticmethod
    def pi_2_pi(theta) -> float:
        """
        Returns a normalized angle between [-pi, pi]
        """
        return np.remainder(theta + np.pi, 2 * np.pi) - np.pi
