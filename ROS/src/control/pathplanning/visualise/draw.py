import math
from typing import List

from bezier import BezierPoint, get_bezier_curve_iterator
from PyQt5 import QtCore as QtC
from PyQt5 import QtGui as QtG


class Draw:
    POINT_SIZE = 0.2
    RASTER_WIDTH = 3

    def __init__(self, widget):
        self.widget = widget

    def draw_grid(self, painter: QtG.QPainter):
        painter.setPen(QtG.QPen(QtG.QColor(0, 0, 0)))
        vertical_line_count = int(
            self.widget.width() / self.widget.zoom_level / self.RASTER_WIDTH + 1
        )
        horizontal_line_count = int(
            self.widget.height() / self.widget.zoom_level / self.RASTER_WIDTH + 1
        )

        px_width = self.widget.zoom_level * self.RASTER_WIDTH

        painter.drawLine(
            QtC.QPointF(self.widget.width() // 2, 0),
            QtC.QPointF(self.widget.width() // 2, self.widget.height()),
        )
        for line_index in range((vertical_line_count + 1) // 2):
            painter.drawLine(
                QtC.QPointF(self.widget.width() // 2 + line_index * px_width, 0),
                QtC.QPointF(
                    self.widget.width() // 2 + line_index * px_width,
                    self.widget.height(),
                ),
            )
            painter.drawLine(
                QtC.QPointF(self.widget.width() // 2 - line_index * px_width, 0),
                QtC.QPointF(
                    self.widget.width() // 2 - line_index * px_width,
                    self.widget.height(),
                ),
            )

        painter.drawLine(
            QtC.QPointF(0, self.widget.height() // 2),
            QtC.QPointF(self.widget.width(), self.widget.height() // 2),
        )
        for line_index in range((horizontal_line_count + 1) // 2):
            painter.drawLine(
                QtC.QPointF(0, self.widget.height() // 2 + line_index * px_width),
                QtC.QPointF(
                    self.widget.width(),
                    self.widget.height() // 2 + line_index * px_width,
                ),
            )
            painter.drawLine(
                QtC.QPointF(0, self.widget.height() // 2 - line_index * px_width),
                QtC.QPointF(
                    self.widget.width(),
                    self.widget.height() // 2 - line_index * px_width,
                ),
            )

    def draw_unvisible_cones(
        self, cones: List[QtC.QPointF], painter: QtG.QPainter, color: QtG.QColor
    ):
        # Draw a circle at each cone
        color.setAlpha(50)
        painter.setPen(QtG.QPen(color))
        painter.setBrush(QtG.QBrush(color))
        diameter = self.widget.CONE_SIZE * self.widget.zoom_level
        for cone in cones:
            screen_pos = self.widget.coordinateToScreen(cone)
            circle_rect = QtC.QRectF(
                screen_pos.x() - diameter / 2,
                screen_pos.y() - diameter / 2,
                diameter,
                diameter,
            )
            painter.drawEllipse(circle_rect)

    def draw_visible_cones(
        self, cones: List[QtC.QPointF], painter: QtG.QPainter, color: QtG.QColor
    ):
        # Draw a circle at each visible cone
        painter.setPen(QtG.QPen(color))
        painter.setBrush(QtG.QBrush(color))
        diameter = self.widget.CONE_SIZE * self.widget.zoom_level
        for cone in cones:
            screen_pos = self.widget.coordinateToScreen(cone)
            circle_rect = QtC.QRectF(
                screen_pos.x() - diameter / 2,
                screen_pos.y() - diameter / 2,
                diameter,
                diameter,
            )
            painter.drawEllipse(circle_rect)

    def draw_indexes(
        self, cones: List[QtC.QPointF], painter: QtG.QPainter, color: QtG.QColor
    ):
        pen = QtG.QPen(color, self.widget.CONE_SIZE * self.widget.zoom_level)
        painter.setPen(pen)
        diameter = self.widget.CONE_SIZE * self.widget.zoom_level
        painter.setFont(
            QtG.QFont(
                "Arial",
                int(0.4 * diameter),
            )
        )
        for index, cone in enumerate(cones):
            screen_pos = self.widget.coordinateToScreen(cone)
            text_rect = QtC.QRectF(
                screen_pos.x() - diameter,
                screen_pos.y() - diameter,
                2 * diameter,
                2 * diameter,
            )
            painter.drawText(text_rect, QtC.Qt.AlignCenter, str(index))

    def draw_cones(self, painter: QtG.QPainter):
        self.draw_unvisible_cones(
            self.widget.blue_cones, painter, QtG.QColor(QtC.Qt.blue)
        )
        self.draw_unvisible_cones(
            self.widget.yellow_cones, painter, QtG.QColor(QtC.Qt.yellow)
        )
        self.draw_visible_cones(
            self.widget.selected_blue_cones, painter, QtG.QColor(QtC.Qt.blue)
        )
        self.draw_visible_cones(
            self.widget.selected_yellow_cones, painter, QtG.QColor(QtC.Qt.yellow)
        )
        self.draw_visible_cones(
            self.widget.orange_cones, painter, QtG.QColor(255, 165, 0)
        )
        self.draw_indexes(self.widget.blue_cones, painter, QtG.QColor(QtC.Qt.white))
        self.draw_indexes(self.widget.yellow_cones, painter, QtG.QColor(QtC.Qt.black))
        self.draw_indexes(self.widget.orange_cones, painter, QtG.QColor(QtC.Qt.white))

    def draw_bezier_line(
        self, bezierPoints: List[BezierPoint], painter: QtG.QPainter, color: QtG.QColor
    ):
        if bezierPoints != []:
            painter.setPen(QtG.QPen(color, 2.0))
            painter.setBrush(QtG.QBrush())
            path = QtG.QPainterPath()
            path.moveTo(self.widget.coordinateToScreen(bezierPoints[0].m))

            for current_point, next_point in get_bezier_curve_iterator(
                bezierPoints, self.widget.is_closed
            ):
                path.cubicTo(
                    self.widget.coordinateToScreen(current_point.c2),
                    self.widget.coordinateToScreen(next_point.c1),
                    self.widget.coordinateToScreen(next_point.m),
                )

                painter.drawPath(path)
        return

    def draw_points(
        self, points: List[QtC.QPointF], painter: QtG.QPainter, color: QtG.QColor
    ):
        painter.setPen(QtG.QPen(color))
        painter.setBrush(QtG.QBrush(color))
        diameter = self.POINT_SIZE * self.widget.zoom_level
        for cone in points:
            screen_pos = self.widget.coordinateToScreen(cone)
            circle_rect = QtC.QRectF(
                screen_pos.x() - diameter / 2,
                screen_pos.y() - diameter / 2,
                diameter,
                diameter,
            )
            painter.drawEllipse(circle_rect)

    def draw_line(
        self,
        points: List[QtC.QPointF],
        painter: QtG.QPainter,
        line_color: QtG.QColor,
        point_color: QtG.QColor,
    ):
        if points is not None and len(points) > 0:
            painter.setPen(QtG.QPen(line_color, 3.0))
            painter.setBrush(QtG.QBrush())
            for index, pathPoint in enumerate(points):
                screen_pos = self.widget.coordinateToScreen(pathPoint)
                if index == 0:
                    start = self.widget.coordinateToScreen(self.widget.car_pos)
                else:
                    start = self.widget.coordinateToScreen(points[index - 1])
                end = screen_pos
                painter.drawLine(start, end)

            painter.setPen(QtG.QPen(point_color))
            painter.setBrush(QtG.QBrush(point_color))
            diameter = self.POINT_SIZE * self.widget.zoom_level
            for pathPoint in points:
                screen_pos = self.widget.coordinateToScreen(pathPoint)
                circle_rect = QtC.QRectF(
                    screen_pos.x() - diameter / 2,
                    screen_pos.y() - diameter / 2,
                    diameter,
                    diameter,
                )
                painter.drawEllipse(circle_rect)

    def draw_car(self, painter):
        painter.setPen(QtG.QPen(QtC.Qt.green))
        painter.setBrush(QtG.QBrush(QtC.Qt.green))
        screen_pos = self.widget.coordinateToScreen(self.widget.car_pos)
        diameter = self.widget.CAR_POINT_SIZE * self.widget.zoom_level
        circle_rect = QtC.QRectF(
            screen_pos.x() - diameter / 2,
            screen_pos.y() - diameter / 2,
            diameter,
            diameter,
        )
        painter.drawEllipse(circle_rect)
        painter.setPen(QtG.QPen(QtC.Qt.red))
        painter.setBrush(QtG.QBrush(QtC.Qt.red))
        screen_pos = self.widget.coordinateToScreen(
            self.widget.car_pos
            + (self.widget.CAR_POINT_SIZE / 2)
            * QtC.QPointF(math.cos(self.widget.car_rot), math.sin(self.widget.car_rot))
        )
        diameter = self.widget.CAR_HANDLE_SIZE * self.widget.zoom_level
        circle_rect = QtC.QRectF(
            screen_pos.x() - diameter / 2,
            screen_pos.y() - diameter / 2,
            diameter,
            diameter,
        )
        painter.drawEllipse(circle_rect)

    def draw_scale(self, painter: QtG.QPainter):
        painter.setFont(QtG.QFont("Serif", 12))
        painter.setPen(QtC.Qt.black)
        text = "Raster breedte:   " + str(self.RASTER_WIDTH) + " meter"
        x = 20
        y = 30
        painter.drawText(x, y, text)

    def draw_pathnr(self, painter: QtG.QPainter):
        font = QtG.QFont("Serif", 20)
        fm = QtG.QFontMetrics(font)
        painter.setFont(QtG.QFont("Serif", 20))
        painter.setPen(QtC.Qt.black)
        text = f"Path {self.widget.pathnr + 1} of {self.widget.nr_paths}"
        text_width = fm.width(text)
        x = self.widget.width() // 2 - text_width // 2
        y = 40
        painter.drawText(x, y, text)
