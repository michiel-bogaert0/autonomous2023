from dataclasses import dataclass
from typing import Dict

from PyQt5 import QtCore as QtC


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
