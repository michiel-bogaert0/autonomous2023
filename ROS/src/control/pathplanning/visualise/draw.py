from typing import List

from PyQt5 import QtCore as QtC
from PyQt5 import QtGui as QtG

# from PyQt5 import QtWidgets as QtW


class Draw:
    def __init__(self, widget):
        self.widget = widget

    def draw_grid(self, painter: QtG.QPainter):
        painter.setPen(QtG.QPen(QtG.QColor(0, 0, 0)))
        vertical_line_count = int(
            self.widget.width() / self.widget.zoom_level / self.widget.RASTER_WIDTH + 1
        )
        horizontal_line_count = int(
            self.widget.height() / self.widget.zoom_level / self.widget.RASTER_WIDTH + 1
        )

        px_width = self.widget.zoom_level * self.widget.RASTER_WIDTH

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
        pen = QtG.QPen(color, self.widget.CONE_SIZE * self.widget.zoom_level)
        painter.setPen(pen)
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
        pen = QtG.QPen(color, self.widget.CONE_SIZE * self.widget.zoom_level)
        painter.setPen(pen)
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
        font = QtG.QFont(
            "Arial",
            int(0.75 * diameter),
        )
        painter.setFont(font)
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
