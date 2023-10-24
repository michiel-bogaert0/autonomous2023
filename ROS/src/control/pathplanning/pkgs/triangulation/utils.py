from typing import List, Tuple

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
        return np.empty(0)

    ind = np.argsort(distances_squared)
    return points_within_distance[ind, :]


def extend_line_to_rectangle(point, angle_radians, length, width):
    """
    Calculates a rectangular region by inflating a line starting at (point, angle) with given length and width.

    Args:
        point: starting point
        angle_radians: angle of the line that starts from 'point' and has to be inflated
        length: the length of the line
        width: the width of the rectangle. Line will be inflated width/2 on each side.

    Returns:
        the four points of the rectangle
    """
    x, y = point

    # Calculate the endpoint of the line using the given angle and length
    end_x = x + length * np.cos(angle_radians)
    end_y = y + length * np.sin(angle_radians)

    # Calculate the perpendicular vectors to get the points of the rectangle
    # We'll assume a width of 1 (you can adjust this if you want a wider rectangle)
    dx = width * np.cos(angle_radians + np.pi / 2)
    dy = width * np.sin(angle_radians + np.pi / 2)

    # Calculate the four points of the rectangle
    p1 = (x + dx, y + dy)
    p2 = (end_x + dx, end_y + dy)
    p3 = (end_x - dx, end_y - dy)
    p4 = (x - dx, y - dy)

    return p1, p2, p3, p4


def vectorized_is_point_inside_rectangle(rectangle_points, points):
    """
    Checks in a vectorized manner if a list of points is inside a rectangle determined as 4 corner points (see function above)

    Args:
        rectangle_points: corner points of rectangular region
        points: list of points to check

    Returns:
        List of bools. True when inside region, False otherwise
    """
    p1, p2, p3, p4 = rectangle_points

    def cross_product(p1, p2, p3):
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

    def is_point_on_same_side(p1, p2, p3, p4, point):
        return (
            cross_product(p1, p2, point) * cross_product(p3, p4, point) >= 0
            and cross_product(p2, p3, point) * cross_product(p4, p1, point) >= 0
        )

    result = np.empty(len(points), dtype=bool)
    for i, point in enumerate(points):
        result[i] = is_point_on_same_side(p1, p2, p3, p4, point)

    return result


def is_point_inside_rectangle(rectangle_points, point):
    """
    Given the four points of the rectangle and a point, we'll check if the point is inside the rectangle.

    A point P(x, y) lies inside a rectangle defined by four points if:
    1. P is on the same side of each line segment formed by any two consecutive points of the rectangle.
    2. The sum of the angles formed by (P, p1, p2), (P, p2, p3), (P, p3, p4), and (P, p4, p1) is 360 degrees.

    Args:
        rectangle_points: corner points of rectangular region
        point: point to check

    Returns:
        boolean. True if inside region, False otherwise
    """

    p1, p2, p3, p4 = rectangle_points

    def cross_product(p1, p2, p3):
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

    def is_point_on_same_side(p1, p2, p3, p4, point):
        return (
            cross_product(p1, p2, point) * cross_product(p3, p4, point) >= 0
            and cross_product(p2, p3, point) * cross_product(p4, p1, point) >= 0
        )

    return is_point_on_same_side(p1, p2, p3, p4, point) and is_point_on_same_side(
        p2, p3, p4, p1, point
    )


def check_if_feasible_child(
    parent: Node,
    path: List[Tuple[float, float]],
    next_pos: np.ndarray,
    bad_points: np.ndarray,
    center_points: np.ndarray,
    cones: np.ndarray,
    max_angle_change: float,
    safety_dist_squared: float,
    rect_width: float,
    bad_points_threshold: int,
    center_points_threshold: int,
):
    """
    Checks if a certain child node is feasible to be added to a path where 'parent' is the current leaf node.

    Args:
        parent: current path 'leaf'
        path: list of (x, y) positions of points already on the path.
        next_pos: the child node of which the feasibility needs to be checked
        bad_points: the points of triangulation that were filtered out
        center_points: the points of triangulation that are deemed being part of the centerline
        cones: list of original cones
        max_angle_change: maximal angle change from parent to child
        safety_dist_squared: squared safety distance used for cone collision avoidance
        rect_width: width of the rectangle for line expansion
        bad_points_threshold: threshold for the amount of bad points crossings allowed
        center_points_threshold: threshold for the amount of center point (not used) crossings allowed
    """

    # Make sure we're not looping from the parent to itself
    if np.isclose(next_pos[0], parent.x) and np.isclose(next_pos[1], parent.y):
        return False

    # Make sure we are not visiting nodes double
    if (next_pos[0], next_pos[1]) in path:
        return False

    angle_node = np.arctan2(next_pos[1] - parent.y, next_pos[0] - parent.x)
    angle_change = angle_node - parent.angle
    distance_node = (parent.x - next_pos[0]) ** 2 + (parent.y - next_pos[1]) ** 2

    abs_angle_change = min(abs(angle_change), 2 * np.pi - abs(angle_change))

    # Check that the angle change is within bounds
    if abs_angle_change > max_angle_change:
        return False

    # Check we're not colliding with a cone
    if not no_collision(parent, next_pos, cones, safety_dist_squared):
        return False

    # Don't allow to come close to bad points
    rectangle_points = extend_line_to_rectangle(
        (parent.x, parent.y), angle_node, distance_node ** (1 / 2), rect_width
    )
    result = vectorized_is_point_inside_rectangle(rectangle_points, bad_points)
    if sum(result) > bad_points_threshold:
        return False

    # Also don't allow to skip center points
    result = vectorized_is_point_inside_rectangle(rectangle_points, center_points)
    if sum(result) > center_points_threshold:
        return False

    return True
