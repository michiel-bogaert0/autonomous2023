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
