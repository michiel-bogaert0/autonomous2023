import numpy as np
from scipy.spatial import Delaunay, distance
from typing import Tuple
import triangulation.utils as utils


def get_center_points(
    position_cones: np.ndarray,
    classes,
    triangulation_min_var: float,
    triangulation_var_threshold: float,
    range_front: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Get center points of the edges of the triangles

    Also perform a preliminary filtering on these points to remove useless points.

    Args:
        position_cones: position of the cones
        triangulation_min_var: the minimum variance each allowed set of triangle edge lengths can always have.
            So it's the minimal maximum variance
        triangulation_var_threshold: Factor multiplied to the median of the variance of triangle lengths in order to filter bad triangles
        range_front: The lookahead distance for sorting points

    Returns:
        Tuple of center_points, duplicated_centers, triangles
    """
    tri = Delaunay(position_cones)
    indices = tri.simplices
    triangles = position_cones[indices]
    triangle_points_classes = classes[indices]

    # The distances of all three sides of each triangle
    distances = triangles - np.roll(triangles, 1, axis=1)
    distances = np.power(distances[:, :, 0], 2) + np.power(distances[:, :, 1], 2)
    variances = np.var(
        distances, axis=1
    )  # The variance between the side lengths of each triangle

    # We assume the triangles between the cones on the racing path to be much more equilateral than the useless triangles
    # We use the median variance to have a dynamic threshold
    var_filter = max(
        triangulation_min_var, np.median(variances) * triangulation_var_threshold
    )
    triangles = triangles[variances < var_filter, :]
    triangle_points_classes = triangle_points_classes[variances < var_filter]

    # Find all center points
    center_points = (triangles + np.roll(triangles, 1, axis=1)) / 2
    
    # Find center point classes and filter out bad ones NOW
    center_point_classes = ( triangle_points_classes + np.roll(triangle_points_classes, 1, axis=1) ) / 2
    bad_points = center_points[np.logical_not(np.logical_xor( center_point_classes != center_point_classes.astype(int), center_point_classes == 1.5 ))]
    center_points = center_points[np.logical_xor( center_point_classes != center_point_classes.astype(int), center_point_classes == 1.5 )]

    flattened_center_points = center_points.reshape((center_points.size // 2, 2))

    # Each center point inside of the racing track should have been added as part of two neighbouring triangles
    # These duplicated centers should form the raceline (with some odd exceptions that can be filtered out using the tree filtering strategy)
    unique, counts = np.unique(flattened_center_points, axis=0, return_counts=True)
    duplicated_centers = unique[counts > 1]

    # Add closest center in front of you as this one will not be duplicated
    closest_centers = utils.sort_closest_to(unique, (0,0), range_front)
    duplicated_centers = np.append(duplicated_centers, [closest_centers[0]], axis=0)

    return center_points, duplicated_centers, triangles, bad_points