import copy
import math
from dataclasses import dataclass
from typing import Dict, Iterator, List, Tuple

from PyQt5 import QtCore as QtC

STEP = 1
BEZIER_CONTROL = 0.35  # ziet er redelijk smooth uit zo


@dataclass
class BezierPoint:
    def __init__(self, c1: QtC.QPointF, m: QtC.QPointF, c2: QtC.QPointF):
        self.m = m
        self._control_points = [c1, c2]

    @property
    def c1(self) -> QtC.QPointF:
        return self._control_points[0]

    @property
    def c2(self) -> QtC.QPointF:
        return self._control_points[1]

    def to_dict(self, offset: QtC.QPointF = QtC.QPointF(0, 0)) -> Dict:
        return {
            "C1": [(self.c1 + offset).x(), (self.c1 + offset).y()],
            "M": [(self.m + offset).x(), (self.m + offset).y()],
            "C2": [(self.c2 + offset).x(), (self.c2 + offset).y()],
        }


def make_bezier(points: List[QtC.QPointF]) -> List[BezierPoint]:
    bezier = []
    if len(points) > 2:
        for i, m in enumerate(points[::STEP]):
            if STEP * i == len(points) - 1:
                next = points[0]
            else:
                next = points[STEP * i + 1]

            if i == 0:
                prev = points[-1]
            else:
                prev = points[STEP * i - 1]

            rico = next - prev
            ricoN = rico / math.sqrt(rico.x() ** 2 + rico.y() ** 2)

            l2 = next - m
            dist2 = math.sqrt(l2.x() ** 2 + l2.y() ** 2)

            l1 = m - prev
            dist1 = math.sqrt(l1.x() ** 2 + l1.y() ** 2)

            c2 = m + ricoN * BEZIER_CONTROL * dist2
            c1 = m - ricoN * BEZIER_CONTROL * dist1

            bezier.append(BezierPoint(c1, m, c2))
    return bezier


def make_refline(
    blue_cones: List[QtC.QPointF], yellow_cones: List[QtC.QPointF]
) -> List[QtC.QPointF]:
    refPoints = []
    c = 0
    for i, yellow_cone in enumerate(yellow_cones):
        min_distance = math.inf
        nearest_blue = None
        removed = 0
        for j, blue_cone in enumerate(blue_cones[c:]):
            distance = math.sqrt(
                (blue_cone.x() - yellow_cone.x()) ** 2
                + (blue_cone.y() - yellow_cone.y()) ** 2
            )
            if distance < min_distance:
                min_distance = distance
                for index, el in enumerate(blue_cones[c + removed : c + j]):
                    if index + removed > 0 or i == 0:
                        mindis = math.inf
                        closest = None
                        for yc in yellow_cones[: i + 1]:
                            dis = math.sqrt(
                                (el.x() - yc.x()) ** 2 + (el.y() - yc.y()) ** 2
                            )
                            if dis < mindis:
                                mindis = dis
                                closest = yc
                        refPoints.append((el + closest) / 2)
                removed = j
                nearest_blue = blue_cone
        if nearest_blue:
            refPoints.append((yellow_cone + nearest_blue) / 2)

        c += removed

    for index, el in enumerate(blue_cones[c:]):
        if index > 0:  # or i == 0
            mindis = math.inf
            closest = None
            for yc in yellow_cones:
                dis = math.sqrt((el.x() - yc.x()) ** 2 + (el.y() - yc.y()) ** 2)
                if dis < mindis:
                    mindis = dis
                    closest = yc
            if closest is not None:
                refPoints.append((el + closest) / 2)
    return refPoints


def get_bezier_curve_iterator(
    bezierPoints: List[BezierPoint],
) -> Iterator[Tuple[BezierPoint, BezierPoint]]:
    if len(bezierPoints) == 0:
        return iter(())
    shifted_by_one = copy.deepcopy(bezierPoints)
    first_item = shifted_by_one.pop(0)
    shifted_by_one += [first_item]

    return zip(bezierPoints, shifted_by_one)
