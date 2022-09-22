import numpy as np
from scipy.spatial import Delaunay
from typing import Tuple
import triangulation.utils as utils


def get_center_points(
    position_cones: np.ndarray,
    triangulation_var_threshold: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Get center points of the edges of the triangles

    Also perform a preliminary filtering on these points to remove useless points.

    Args:
        position_cones: position of the cones
        triangulation_var_threshold: Factor multiplied to the median of the variance of triangle lengths in order to filter bad triangles

    Returns:
        Tuple of center_points, duplicated_centers, triangles
    """
    tri = Delaunay(position_cones)
    indices = tri.simplices
    triangles = position_cones[indices]

    # The distances of all three sides of each triangle
    distances = triangles - np.roll(triangles, 1, axis=1)
    distances = np.power(distances[:, :, 0], 2) + np.power(distances[:, :, 1], 2)
    variances = np.var(
        distances, axis=1
    )  # The variance between the side lengths of each triangle

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

    return center_points, duplicated_centers, triangles

def filter_center_points(center_points: np.ndarray, cones: np.ndarray) -> np.ndarray:
    """The center_points will now probably form the center of the racetrack
    Let's try to improve the false positives and false negatives first though

    Args:
        center_points: the probably track center points (Nx2 array)
        cones: array of cones (x, y, colour)
    """

    filtered_points = None

    # False positive removal
    # Remove points who's closest cones have the same colour
    for point in center_points:
        distances_squared = utils.distance_squared(
            point[0], point[1], cones[:, 0], cones[:, 1]
        )
        ind = np.argsort(distances_squared)

        if cones[ind[0], 2] != cones[ind[1], 2]:
            if filtered_points is None:
                filtered_points = np.array(point)
            else:
                filtered_points = np.vstack([filtered_points, point])
    
    # TODO: False negatives

    return filtered_points
