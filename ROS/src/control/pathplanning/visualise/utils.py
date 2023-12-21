import math
from typing import List

import numpy as np
from PyQt5 import QtCore as QtC

# from PyQt5 import QtGui as QtG
# from PyQt5 import QtWidgets as QtW


def real_to_car_transform(
    points: List[QtC.QPointF], car_pos: QtC.QPointF, car_rot: QtC.QPointF
) -> np.ndarray:
    """
    Returns an array of objects that are converted from global reference space to car reference space.
    """
    # Convert the QtC.QPointF points to a 2D NumPy array
    point_array = np.array([[point.x(), point.y()] for point in points])

    orig_pos = np.c_[point_array, np.ones(point_array.shape[0])]

    trans = np.array([[1, 0, -car_pos.x()], [0, 1, -car_pos.y()], [0, 0, 1]])
    rot = np.array(
        [
            [math.cos(car_rot), -math.sin(-car_rot), 0],
            [math.sin(-car_rot), math.cos(car_rot), 0],
            [0, 0, 1],
        ]
    )
    new_pos = (rot @ trans @ orig_pos.T).T

    return new_pos[:, :-1]


def car_to_real_transform(
    objects: np.ndarray, car_pos: QtC.QPointF, car_rot: QtC.QPointF
) -> List[QtC.QPointF]:
    """
    Returns an array of objects that are converted from car reference space to global reference space.
    """
    orig_pos = np.c_[objects, np.ones(objects.shape[0])]

    rot = np.array(
        [
            [math.cos(car_rot), -math.sin(car_rot), 0],
            [math.sin(car_rot), math.cos(car_rot), 0],
            [0, 0, 1],
        ]
    )
    trans = np.array([[1, 0, car_pos.x()], [0, 1, car_pos.y()], [0, 0, 1]])
    new_pos = (trans @ rot @ orig_pos.T).T

    return [QtC.QPointF(coord[0], coord[1]) for coord in new_pos[:, :-1]]


def get_local_poses(
    blue_cones: List[QtC.QPointF],
    yellow_cones: List[QtC.QPointF],
    car_pos: QtC.QPointF,
    car_rot: QtC.QPointF,
) -> np.ndarray:
    yellow_poses = np.empty((0, 3))
    if len(yellow_cones) > 0:
        yellow_poses = np.column_stack(
            (
                real_to_car_transform(yellow_cones, car_pos, car_rot),
                np.ones(len(yellow_cones)),
            )
        )

    blue_poses = np.empty((0, 3))
    if len(blue_cones) > 0:
        blue_poses = np.column_stack(
            (
                real_to_car_transform(blue_cones, car_pos, car_rot),
                np.zeros(len(blue_cones)),
            )
        )

    visible_poses = np.vstack((yellow_poses, blue_poses))
    return visible_poses


def dist(p1: "QtC.QPoint", p2: "QtC.QPoint") -> float:
    def L2Dist(x1: float, y1: float, x2: float, y2: float) -> float:
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    return L2Dist(p1.x(), p1.y(), p2.x(), p2.y())
