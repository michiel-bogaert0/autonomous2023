import numpy as np
from scipy.interpolate import interp1d


class Trajectory:
    """
    Helper class to calculate the cars target point based on a given path it has to follow
    """

    def __init__(self):
        self.path = np.array([])
        self.eucl_dist = np.array([])
        self.closest_index = 0

    def set_path(self, points, current_position):
        """
        Sets the internal path with new points. Expects a numpy array of shape (N, 2)
        Path should be given in world frame

        Finds closest point on path and uses this as anker point.

        Args:
            points: (N,2) numpy array
            current_position: (x, y) of car
        """

        # Interpolate
        distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]
        alpha = np.linspace(0, 1, len(points) * 10)
        interpolator = interp1d(distance, points, kind="linear", axis=0)

        self.path = interpolator(alpha)

        # Calculate closest point index
        self.eucl_dist = np.sqrt(
            (self.path[:, 0] - current_position[0]) ** 2
            + (self.path[:, 1] - current_position[1]) ** 2
        )
        self.closest_index = np.argmin(self.eucl_dist)

    def calculate_target_point(self, minimal_distance, current_position):
        """
        Calculates a target point by traversing the path
        Returns the first points that matches the conditions given by minimal_distance

        Args:
            minimal_distance: lookahead distance
            current_position: Current position of car

        Returns:
            x {float}: x position of target point
            y {float}: y position of target point
            success {bool}: True when target point was found
        """

        # Iterate until found
        found = False
        # i = self.closest_index % len(self.path)

        while not found:
            distance = (current_position[0] - self.path[self.closest_index][0]) ** 2 + (
                current_position[1] - self.path[self.closest_index][1]
            ) ** 2
            if distance > minimal_distance**2:
                return (
                    self.path[self.closest_index][0],
                    self.path[self.closest_index][1],
                    True,
                )

            self.closest_index = (self.closest_index + 1) % len(self.path)
