import numpy as np
from scipy.spatial import Delaunay
from typing import Tuple

def get_center_points(
        position_cones: np.ndarray,
        triangulation_var_threshold: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get center points of the edges of the triangles

        Also perform a preliminary filtering on these points to remove useless points.

        Args:
            position_cones: position of the cones
            triangulation_var_threshold: Factor multiplied to the median of the variance of triangle lengths in order to filter bad triangles

        Returns:
            Tuple of center_points, unique_center_points
        """
        tri = Delaunay(position_cones)
        indices = tri.simplices
        triangles = position_cones[indices]

        # The distances of all three sides of each triangle
        distances = triangles - np.roll(triangles, 1, axis=1)
        distances = np.power(distances[:, :, 0], 2) + np.power(distances[:, :, 1], 2)
        variances = np.var(distances, axis=1)  # The variance between the side lengths of each triangle

        # We assume the triangles between the cones on the racing path to be much more equilateral than the useless triangles
        var_filter = np.median(variances) * triangulation_var_threshold
        triangles = triangles[variances < var_filter, :]

        # Waiting if frame is okay (Edit by Lucas: idk what this means?)

        # Find all center points
        center_points = (triangles + np.roll(triangles, 1, axis=1)) / 2
        flattened_center_points = center_points.reshape((center_points.size // 2, 2))

        # Each center point inside of the racing track should have been added as part of two neighbouring triangles
        # These duplicated centers should form the raceline (with some odd exceptions that can be filtered out using the tree filtering strategy)
        unique, counts = np.unique(flattened_center_points, axis=0, return_counts=True)
        duplicated_centers = unique[counts > 1]

        return center_points, duplicated_centers