from typing import Tuple
import numpy as np
from pathplanning_dc.node import Node


def distance_squared(x1, y1, x2, y2):
    """Calculate squared distance between 2 points

    Args:
        x1: x-coordinate of first point
        y1: y-coordinate of first point
        x2: x-coordinate of second point
        y2: y-coordinate of second point

    Returns:
    the distance between the points
    """
    return np.power(x1 - x2, 2) + np.power(y1 - y2, 2)


def no_collision(
    parent: Node, point: np.ndarray, cones: np.ndarray, safety_dist_squared
) -> bool:
    """Detect if no collision happened from between path from node and point and safety distance around cones.

    Args:
        parent: Node to start from
        point: Point to check collision path
        cones: cones which can't be collided with
        safety_dist_squared: Safety distance from objects (squared)

    Returns:
    True if no collision happened else False
    """

    # New implementation
    x1 = parent.x
    y1 = parent.y
    x2 = point[0]
    y2 = point[1]
    xc = cones[:, 0]
    yc = cones[:, 1]

    t_numerator = (x1 - xc) * (x1 - x2) + (y1 - yc) * (y1 - y2)
    t_denominator = (x1 - x2) ** 2 + (y1 - y2) ** 2

    t = t_numerator / t_denominator

    t[t < 0] = 0
    t[t > 1] = 1

    xp = x1 + (x2 - x1) * t
    yp = y1 + (y2 - y1) * t

    dist_squared = distance_squared(xc, yc, xp, yp)

    return np.all(dist_squared >= safety_dist_squared)


def get_closest_center(
    center_points: np.ndarray,
    amount: int,
    origin: Tuple[float, float] = (0, 0),
    max_distance: float = -1,
) -> np.ndarray:
    """Get closest center points to origin

    Args:
        center_points: All center points
        amount: Amount of closest center points to extract
        origin: (optional) (x, y) position of the origin point to measure from,
            root by default
        max_distance: (optional) only return cones closer than this distance (-1 to disable)

    Returns:
    array of closest center points with length "amount"
    """
    distances_squared = distance_squared(
        origin[0], origin[1], center_points[:, 0], center_points[:, 1]
    )

    if max_distance == -1:
        points_within_distance = center_points
    else:
        points_within_distance = center_points[distances_squared < max_distance**2]

    # If there are not enough points
    if points_within_distance.shape[0] <= amount:
        return points_within_distance

    ind = np.argpartition(distances_squared, amount)
    return points_within_distance[ind[:amount]]


def sort_closest_to(
    center_points: np.ndarray,
    origin: Tuple[float, float] = (0, 0),
    max_distance: float = -1,
) -> np.ndarray:
    """Sorts the array of center points according to their distance to the origin (ascending)

    Args:
        center_points: All center points
        origin: (optional) (x, y) position of the origin point to measure from,
            root by default
        max_distance: (optional) only return cones closer than this distance (-1 to disable)

    Returns: array of center points ordered by increasing distance to the origin
    """
    distances_squared = distance_squared(
        origin[0], origin[1], center_points[:, 0], center_points[:, 1]
    )

    points_within_distance = center_points[distances_squared < max_distance**2]
    distances_squared = distances_squared[distances_squared < max_distance**2]
    if points_within_distance.shape[0] == 0:
        return np.empty()

    ind = np.argsort(distances_squared)
    return points_within_distance[ind, :]
