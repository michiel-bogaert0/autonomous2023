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


def get_closest_center(center_points: np.ndarray, amount: int) -> np.ndarray:
    """Get closest center points to root

    Args:
        center_points: All unique center points
        amount: Amount of closest center points to extract

    Returns:
    array of closest center points with length "amount"
    """

    # If there are not enough points
    if center_points.shape[0] <= amount:
        return center_points

    distances_squared = distance_squared(0, 0, center_points[:, 0], center_points[:, 1])

    ind = np.argpartition(distances_squared, amount)
    return center_points[ind[:amount]]
