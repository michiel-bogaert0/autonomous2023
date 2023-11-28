import copy
import datetime
import json
import math
import pathlib
from typing import Dict, Iterator, List, Optional, Tuple

import numpy as np
import yaml
from bezierPoint import BezierPoint
from PyQt5 import QtCore as QtC
from PyQt5 import QtGui as QtG
from PyQt5 import QtWidgets as QtW
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class MapWidget(QtW.QFrame):
    def __init__(
        self,
        publisher,
        frame,
        closed=True,
        startpos_x=None,
        startpos_y=None,
        startrot=0,
        yellows=None,
        blues=None,
        oranges=None,
    ):
        super().__init__(None)
        self.setFocus()

        if yellows is None:
            yellows = []
        if blues is None:
            blues = []
        if oranges is None:
            oranges = []

        self.publisher = publisher
        self.frame = frame

        # Create a QVBoxLayout to hold the button and the map
        layout = QtW.QVBoxLayout(self)
        layout.setAlignment(QtC.Qt.AlignTop | QtC.Qt.AlignRight)
        # Set spacing between buttons
        layout.setSpacing(10)

        # Create a QPushButton and add it to the layout
        loopButton = QtW.QPushButton("close/unclose loop", self)
        loopButton.setFixedSize(120, 30)  # Set the size of the button
        layout.addWidget(
            loopButton
        )  # Align the button to the right and top of the layout
        # Create a QPushButton and add it to the layout
        middellineButton = QtW.QPushButton("show/hide middelline", self)
        middellineButton.setFixedSize(120, 30)  # Set the size of the button
        layout.addWidget(
            middellineButton
        )  # Align the button to the right and top of the layout
        # Create a QPushButton and add it to the layout
        selectAllButton = QtW.QPushButton("select all cones", self)
        selectAllButton.setFixedSize(120, 30)  # Set the size of the button
        layout.addWidget(
            selectAllButton
        )  # Align the button to the right and top of the layout
        # Create a QPushButton and add it to the layout
        deselectAllButton = QtW.QPushButton("deselect all cones", self)
        deselectAllButton.setFixedSize(120, 30)  # Set the size of the button
        layout.addWidget(
            deselectAllButton
        )  # Align the button to the right and top of the layout

        # Connect the button's clicked signal to a slot
        loopButton.clicked.connect(self.close_loop_clicked)
        middellineButton.clicked.connect(self.middelline_clicked)
        selectAllButton.clicked.connect(self.select_all_clicked)
        deselectAllButton.clicked.connect(self.deselect_all_clicked)

        # Constants
        self.ZOOM = 1.1
        self.MAX_ZOOM = 10
        self.MIN_ZOOM = 0.1
        self.CAR_POINT_SIZE = 0.5
        self.CAR_HANDLE_SIZE = 15
        self.CONE_SIZE = 0.2
        self.LAYOUT_TYPE = "yaml"

        self.RASTER_WIDTH = 3

        self.STEP = 1
        self.BEZIERPOINT_SIZE = 5
        self.BEZIER_CONTROL = 0.35  # ziet er redelijk smooth uit zo

        self.is_closed = bool(closed)

        self.middelline_on = False

        self.selection: Optional[QtC.QPoint] = None

        # Set the size policy of the MapWidget to Expanding
        self.setSizePolicy(QtW.QSizePolicy.Expanding, QtW.QSizePolicy.Expanding)

        # Set the background color of the MapWidget to white
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QtC.Qt.white)
        self.setPalette(p)

        self.path = None

        # Create a list to store the cones
        self.yellow_cones = yellows
        self.blue_cones = blues
        self.orange_cones = oranges

        self.selected_yellow_cones = []
        self.selected_blue_cones = []

        self.select_all_clicked()

        # Set the initial zoom level
        self.zoom_level = 0.5

        # Define the scale of the coordinate system in pixels per kilometer
        self.pixels_per_km = 100

        # Start position of the car
        if startpos_x is not None:
            self.car_pos: QtC.QPointF = QtC.QPointF(startpos_x, startpos_y)
        else:
            self.car_pos: QtC.QPointF = QtC.QPointF(0, 0)
        self.car_rot: float = startrot
        self.car_rot_handle: QtC.QPointF = self.car_pos + (
            self.CAR_POINT_SIZE / 2
        ) * QtC.QPointF(math.cos(self.car_rot), math.sin(self.car_rot))

        # Set the initial offset in coördinate system to (0, 0)
        self.offset = QtC.QPointF(0, 0)

        # initialize control_pressed
        self.control_pressed = False

        # initialize list of bezier points wich represent the middle of the track
        self.bezierPoints = []

        self.bezier = []

    def close_loop_clicked(self):
        # This method will be called when the button is clicked
        self.is_closed = not self.is_closed
        self.update()

    def middelline_clicked(self):
        # This method will be called when the button is clicked
        self.middelline_on = not self.middelline_on
        self.update()

    def select_all_clicked(self):
        # This method will be called when the button is clicked
        self.selected_blue_cones = []
        self.selected_yellow_cones = []
        for cone in self.yellow_cones:
            self.selected_yellow_cones.append(cone)
        for cone in self.blue_cones:
            self.selected_blue_cones.append(cone)
        self.update()

    def deselect_all_clicked(self):
        # This method will be called when the button is clicked
        self.selected_blue_cones = []
        self.selected_yellow_cones = []
        self.update()

    def real_to_car_transform(self, points: list) -> np.ndarray:
        """
        Returns an array of objects that are converted from global reference space to car reference space.
        """
        # Convert the QtC.QPointF points to a 2D NumPy array
        point_array = np.array([[point.x(), point.y()] for point in points])

        orig_pos = np.c_[point_array, np.ones(point_array.shape[0])]

        trans = np.array(
            [[1, 0, -self.car_pos.x()], [0, 1, -self.car_pos.y()], [0, 0, 1]]
        )
        rot = np.array(
            [
                [math.cos(self.car_rot), -math.sin(-self.car_rot), 0],
                [math.sin(-self.car_rot), math.cos(self.car_rot), 0],
                [0, 0, 1],
            ]
        )
        new_pos = (rot @ trans @ orig_pos.T).T

        return new_pos[:, :-1]

    def car_to_real_transform(self, objects: np.ndarray) -> list:
        """
        Returns an array of objects that are converted from car reference space to global reference space.
        """
        orig_pos = np.c_[objects, np.ones(objects.shape[0])]

        rot = np.array(
            [
                [math.cos(self.car_rot), -math.sin(self.car_rot), 0],
                [math.sin(self.car_rot), math.cos(self.car_rot), 0],
                [0, 0, 1],
            ]
        )
        trans = np.array(
            [[1, 0, self.car_pos.x()], [0, 1, self.car_pos.y()], [0, 0, 1]]
        )
        new_pos = (trans @ rot @ orig_pos.T).T

        return [QtC.QPointF(coord[0], coord[1]) for coord in new_pos[:, :-1]]

    def publish_local_map(self):
        """
        Publishes local map of the selected cones

        """
        cones = self.get_visible_cones()

        local = ObservationWithCovarianceArrayStamped()

        local.header.frame_id = self.frame

        for cone in cones:
            local_ob = ObservationWithCovariance()

            # 100% certainty
            local_ob.covariance = np.zeros(9)
            local_ob.observation.belief = 1

            local_ob.observation.observation_class = int(cone[2])
            local_ob.observation.location.x = cone[0]
            local_ob.observation.location.y = cone[1]

            local.observations.append(local_ob)

        self.publisher.publish(local)

    def get_visible_cones(self) -> np.ndarray:
        yellow_cones = np.empty((0, 3))
        if len(self.selected_yellow_cones) > 0:
            yellow_cones = np.column_stack(
                (
                    self.real_to_car_transform(self.selected_yellow_cones),
                    np.ones(len(self.selected_yellow_cones)),
                )
            )

        blue_cones = np.empty((0, 3))
        if len(self.selected_blue_cones) > 0:
            blue_cones = np.column_stack(
                (
                    self.real_to_car_transform(self.selected_blue_cones),
                    np.zeros(len(self.selected_blue_cones)),
                )
            )

        visible_cones = np.vstack((yellow_cones, blue_cones))
        return visible_cones

    def receive_path(self, rel_path: np.ndarray):
        real_path = self.car_to_real_transform(rel_path)
        self.path = real_path

    def dist(self, p1: "QtC.QPoint", p2: "QtC.QPoint") -> float:
        def L2Dist(x1: float, y1: float, x2: float, y2: float) -> float:
            return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

        return L2Dist(p1.x(), p1.y(), p2.x(), p2.y())

    def get_selected_cone(self, event) -> Optional[QtC.QPoint]:
        # This is in canvas co
        press_location = event.pos()

        if (
            self.dist(press_location, self.coordinateToScreen(self.car_rot_handle))
            < self.CAR_HANDLE_SIZE * self.zoom_level
        ):
            return self.car_rot_handle

        if (
            self.dist(press_location, self.coordinateToScreen(self.car_pos))
            < self.CAR_POINT_SIZE * self.pixels_per_km * self.zoom_level
        ):
            return self.car_pos

        selectables = self.orange_cones + self.blue_cones + self.yellow_cones

        for selectable in selectables:
            control_point_canvas = self.coordinateToScreen(selectable)

            if (
                self.dist(press_location, control_point_canvas)
                < self.CONE_SIZE * self.pixels_per_km * self.zoom_level
            ):
                return selectable

        return None

    # Override the mousePressEvent method to handle mouse click events
    def mousePressEvent(self, event):
        selected_cone = self.get_selected_cone(event)
        if selected_cone is not None:
            # Remove a cone with SHIFT + click
            if event.modifiers() & QtC.Qt.ShiftModifier:
                if selected_cone in self.yellow_cones:
                    self.yellow_cones.remove(selected_cone)
                    if selected_cone in self.selected_yellow_cones:
                        self.selected_yellow_cones.remove(selected_cone)
                elif selected_cone in self.blue_cones:
                    self.blue_cones.remove(selected_cone)
                    if selected_cone in self.selected_blue_cones:
                        self.selected_blue_cones.remove(selected_cone)
                elif selected_cone in self.orange_cones:
                    self.orange_cones.remove(selected_cone)
                self.update()
            elif event.modifiers() == QtC.Qt.AltModifier:
                if selected_cone in self.yellow_cones:
                    if selected_cone in self.selected_yellow_cones:
                        self.selected_yellow_cones.remove(selected_cone)
                    else:
                        self.selected_yellow_cones.append(selected_cone)
                elif selected_cone in self.blue_cones:
                    if selected_cone in self.selected_blue_cones:
                        self.selected_blue_cones.remove(selected_cone)
                    else:
                        self.selected_blue_cones.append(selected_cone)
                self.update()

            # Drag a cone or car_handle
            else:
                self.selection = selected_cone
                self.selected_location = event.pos()
                self.drag_start_pos = event.pos()
        # orange cones with alt key
        elif event.modifiers() == QtC.Qt.AltModifier:
            self.orange_cones.append(self.screenToCoordinate(event.pos()))
            self.update()
        # Drag the screen
        elif event.modifiers() & QtC.Qt.ControlModifier:
            self.control_pressed = True
            self.drag_start_pos = event.pos()
        # Place a yellow cone
        elif event.button() == QtC.Qt.LeftButton and not (
            event.modifiers() & QtC.Qt.ShiftModifier
        ):
            # Add a visual point to the list at the position where the user clicked
            point = self.screenToCoordinate(event.pos())
            self.yellow_cones.append(point)
            self.selected_yellow_cones.append(point)
            # Trigger a repaint of the MapWidget to update the visual points
            self.update()
        # Place a blue cone
        elif event.button() == QtC.Qt.RightButton and not (
            event.modifiers() & QtC.Qt.ShiftModifier
        ):
            # Add a visual point to the list at the position where the user clicked
            point = self.screenToCoordinate(event.pos())
            self.blue_cones.append(point)
            self.selected_blue_cones.append(point)
            # Trigger a repaint of the MapWidget to update the visual points
            self.update()

    def mouseDoubleClickEvent(self, event):
        if event.button() == QtC.Qt.LeftButton:
            selected_cone = self.get_selected_cone(event)
            if selected_cone is not None:
                if selected_cone in self.yellow_cones[:-1]:
                    point = selected_cone + QtC.QPointF(0.10, 0.10)
                    index = self.yellow_cones.index(selected_cone)
                    self.yellow_cones.insert(index, point)
                    self.selected_yellow_cones.append(point)
                elif selected_cone in self.blue_cones[:-1]:
                    point = selected_cone + QtC.QPointF(0.10, 0.10)
                    index = self.blue_cones.index(selected_cone)
                    self.blue_cones.insert(index, point)
                    self.selected_blue_cones.append(point)
                self.update()

    def mouseReleaseEvent(self, event):
        # Reset the flag and the drag start position
        self.selection = None
        self.control_pressed = False
        self.drag_start_pos = None
        self.update_car()

        # the only time you're sure that you don't get a transformation fault in the path
        self.update()

    def update_car(self):
        self.car_rot_handle: QtC.QPointF = self.car_pos + (
            self.CAR_POINT_SIZE / 2
        ) * QtC.QPointF(math.cos(self.car_rot), math.sin(self.car_rot))
        return None

    def mouseMoveEvent(self, event):
        # Drag the selected cone or car_handle
        if self.selection is not None:
            if self.selection == self.car_rot_handle:
                self.car_rot = math.atan2(
                    self.screenToCoordinate(event.pos()).y() - self.car_pos.y(),
                    self.screenToCoordinate(event.pos()).x() - self.car_pos.x(),
                )
            else:
                drag_distance = self.screenToCoordinate(
                    event.pos()
                ) - self.screenToCoordinate(self.drag_start_pos)
                self.selection += drag_distance  # QtC.QPointF(drag_distance)/(self.pixels_per_km * self.zoom_level)
                self.drag_start_pos = event.pos()
            self.update()

        # If the Control key is pressed and the mouse is dragged, move the map
        elif self.control_pressed and self.drag_start_pos is not None:
            # Calculate the distance between the current position and the drag start position
            drag_distance = self.screenToCoordinate(
                event.pos()
            ) - self.screenToCoordinate(self.drag_start_pos)
            # Update the offset in coordinate system
            self.offset -= drag_distance
            # Save the current position as the new drag start position
            self.drag_start_pos = event.pos()
            # Trigger a repaint of the MapWidget to update the visual
            self.update()

    # Override the paintEvent method to draw the visual points
    def paintEvent(self, event):
        self.publish_local_map()

        painter = QtG.QPainter(self)
        self.draw_grid(painter)
        # Draw a circle at each visual point
        for _index, cone in enumerate(self.yellow_cones):
            color = QtG.QColor(QtC.Qt.yellow)
            color.setAlpha(50)
            pen = QtG.QPen(color, self.CONE_SIZE * self.pixels_per_km * self.zoom_level)
            painter.setPen(pen)
            screen_pos = self.coordinateToScreen(cone)
            diameter = self.CONE_SIZE * self.pixels_per_km * self.zoom_level
            circle_rect = QtC.QRectF(
                screen_pos.x() - diameter / 2,
                screen_pos.y() - diameter / 2,
                diameter,
                diameter,
            )
            painter.drawEllipse(circle_rect)

        # Draw a circle at each visual point
        for _index, cone in enumerate(self.blue_cones):
            color = QtG.QColor(QtC.Qt.blue)
            color.setAlpha(50)
            pen = QtG.QPen(color, self.CONE_SIZE * self.pixels_per_km * self.zoom_level)
            painter.setPen(pen)
            screen_pos = self.coordinateToScreen(cone)
            diameter = self.CONE_SIZE * self.pixels_per_km * self.zoom_level
            circle_rect = QtC.QRectF(
                screen_pos.x() - diameter / 2,
                screen_pos.y() - diameter / 2,
                diameter,
                diameter,
            )
            painter.drawEllipse(circle_rect)

        # Draw a circle at each visual point
        for _index, cone in enumerate(self.selected_yellow_cones):
            color = QtG.QColor(QtC.Qt.yellow)
            pen = QtG.QPen(color, self.CONE_SIZE * self.pixels_per_km * self.zoom_level)
            painter.setPen(pen)
            screen_pos = self.coordinateToScreen(cone)
            diameter = self.CONE_SIZE * self.pixels_per_km * self.zoom_level
            circle_rect = QtC.QRectF(
                screen_pos.x() - diameter / 2,
                screen_pos.y() - diameter / 2,
                diameter,
                diameter,
            )
            painter.drawEllipse(circle_rect)

        # Draw a circle at each visual point
        for _index, cone in enumerate(self.selected_blue_cones):
            color = QtG.QColor(QtC.Qt.blue)
            pen = QtG.QPen(color, self.CONE_SIZE * self.pixels_per_km * self.zoom_level)
            painter.setPen(pen)
            screen_pos = self.coordinateToScreen(cone)
            diameter = self.CONE_SIZE * self.pixels_per_km * self.zoom_level
            circle_rect = QtC.QRectF(
                screen_pos.x() - diameter / 2,
                screen_pos.y() - diameter / 2,
                diameter,
                diameter,
            )
            painter.drawEllipse(circle_rect)

        # Give index to visual point
        for index, cone in enumerate(self.yellow_cones):
            screen_pos = self.coordinateToScreen(cone)
            pen = QtG.QPen(
                QtC.Qt.black, self.CONE_SIZE * self.pixels_per_km * self.zoom_level
            )
            painter.setPen(pen)
            font = QtG.QFont(
                "Arial",
                int(0.75 * self.CONE_SIZE * self.pixels_per_km * self.zoom_level),
            )
            painter.setFont(font)
            text_rect = QtC.QRectF(
                screen_pos.x() - diameter,
                screen_pos.y() - diameter,
                2 * diameter,
                2 * diameter,
            )
            painter.drawText(text_rect, QtC.Qt.AlignCenter, str(index))

        # Give index to visual point
        for index, cone in enumerate(self.blue_cones):
            screen_pos = self.coordinateToScreen(cone)
            pen = QtG.QPen(
                QtC.Qt.white, self.CONE_SIZE * self.pixels_per_km * self.zoom_level
            )
            painter.setPen(pen)
            font = QtG.QFont(
                "Arial",
                int(0.75 * self.CONE_SIZE * self.pixels_per_km * self.zoom_level),
            )
            painter.setFont(font)
            text_rect = QtC.QRectF(
                screen_pos.x() - diameter,
                screen_pos.y() - diameter,
                2 * diameter,
                2 * diameter,
            )
            painter.drawText(text_rect, QtC.Qt.AlignCenter, str(index))

        for index, cone in enumerate(self.orange_cones):
            pen = QtG.QPen(
                QtG.QColor(255, 165, 0),
                self.CONE_SIZE * self.pixels_per_km * self.zoom_level,
            )
            painter.setPen(pen)
            screen_pos = self.coordinateToScreen(cone)
            diameter = self.CONE_SIZE * self.pixels_per_km * self.zoom_level
            circle_rect = QtC.QRectF(
                screen_pos.x() - diameter / 2,
                screen_pos.y() - diameter / 2,
                diameter,
                diameter,
            )
            painter.drawEllipse(circle_rect)

            pen = QtG.QPen(
                QtC.Qt.white, self.CONE_SIZE * self.pixels_per_km * self.zoom_level
            )
            painter.setPen(pen)
            font = QtG.QFont(
                "Arial",
                int(0.75 * self.CONE_SIZE * self.pixels_per_km * self.zoom_level),
            )
            painter.setFont(font)
            text_rect = QtC.QRectF(
                screen_pos.x() - diameter,
                screen_pos.y() - diameter,
                2 * diameter,
                2 * diameter,
            )
            painter.drawText(text_rect, QtC.Qt.AlignCenter, str(index))

        if self.middelline_on:
            self.make_bezier()
            for _index, cone in enumerate(self.bezierPoints):
                pen = QtG.QPen(QtC.Qt.red, self.BEZIERPOINT_SIZE * self.zoom_level)
                painter.setPen(pen)
                screen_pos = self.coordinateToScreen(cone)
                diameter = self.BEZIERPOINT_SIZE * self.zoom_level
                circle_rect = QtC.QRectF(
                    screen_pos.x() - diameter / 2,
                    screen_pos.y() - diameter / 2,
                    diameter,
                    diameter,
                )
                painter.drawEllipse(circle_rect)

            self.paint_bezier_path(painter)

        self.draw_path(painter)

        # Draw the car
        pen = QtG.QPen(
            QtC.Qt.green,
            self.CAR_POINT_SIZE / 2 * self.pixels_per_km * self.zoom_level,
        )
        painter.setPen(pen)
        painter.drawEllipse(
            self.coordinateToScreen(self.car_pos),
            self.CAR_POINT_SIZE / 2 * self.pixels_per_km * self.zoom_level,
            self.CAR_POINT_SIZE / 2 * self.pixels_per_km * self.zoom_level,
        )
        pen = QtG.QPen(QtC.Qt.red, self.CAR_HANDLE_SIZE * self.zoom_level)
        painter.setPen(pen)
        painter.drawEllipse(
            self.coordinateToScreen(
                self.car_pos
                + (self.CAR_POINT_SIZE / 2)
                * QtC.QPointF(math.cos(self.car_rot), math.sin(self.car_rot))
            ),
            self.CAR_HANDLE_SIZE * self.zoom_level,
            self.CAR_HANDLE_SIZE * self.zoom_level,
        )

        # schaal meedelen
        font = QtG.QFont("Serif", 12)
        painter.setFont(font)
        painter.setPen(QtC.Qt.black)
        text = "Raster breedte:   " + str(self.RASTER_WIDTH) + " meter"
        x = 20
        y = 30
        painter.drawText(x, y, text)

        painter.end()

    def draw_path(self, painter):
        if self.path is not None and len(self.path) > 0:
            for index, pathPoint in enumerate(self.path):
                screen_pos = self.coordinateToScreen(pathPoint)
                pen = QtG.QPen(
                    QtC.Qt.green, self.CONE_SIZE * self.pixels_per_km * self.zoom_level
                )
                pen.setWidthF(3.0)
                painter.setPen(pen)
                if index == 0:
                    start = self.coordinateToScreen(self.car_pos)
                else:
                    start = self.coordinateToScreen(self.path[index - 1])
                end = screen_pos
                painter.drawLine(start, end)

                diameter = self.CONE_SIZE * self.pixels_per_km * self.zoom_level / 7
                circle_rect = QtC.QRectF(
                    screen_pos.x() - diameter / 2,
                    screen_pos.y() - diameter / 2,
                    diameter,
                    diameter,
                )
                pen = QtG.QPen(
                    QtC.Qt.red,
                    self.CONE_SIZE * self.pixels_per_km * self.zoom_level / 10,
                )
                brush = QtG.QBrush(QtC.Qt.red)
                painter.setPen(pen)
                painter.setBrush(brush)
                painter.drawEllipse(circle_rect)

    def make_bezier(self):
        self.bezierPoints = []
        c = 0

        for i, yellow_cone in enumerate(self.yellow_cones):
            min_distance = math.inf
            nearest_blue = None
            removed = 0
            for j, blue_cone in enumerate(self.blue_cones[c:]):
                distance = math.sqrt(
                    (blue_cone.x() - yellow_cone.x()) ** 2
                    + (blue_cone.y() - yellow_cone.y()) ** 2
                )
                if distance < min_distance:
                    min_distance = distance
                    for index, el in enumerate(self.blue_cones[c + removed : c + j]):
                        if index + removed > 0 or i == 0:
                            mindis = math.inf
                            closest = None
                            for yc in self.yellow_cones[: i + 1]:
                                dis = math.sqrt(
                                    (el.x() - yc.x()) ** 2 + (el.y() - yc.y()) ** 2
                                )
                                if dis < mindis:
                                    mindis = dis
                                    closest = yc
                            self.bezierPoints.append((el + closest) / 2)
                    removed = j
                    nearest_blue = blue_cone
            if nearest_blue:
                self.bezierPoints.append((yellow_cone + nearest_blue) / 2)

            c += removed

        for index, el in enumerate(self.blue_cones[c:]):
            if index > 0:  # or i == 0
                mindis = math.inf
                closest = None
                for yc in self.yellow_cones:
                    dis = math.sqrt((el.x() - yc.x()) ** 2 + (el.y() - yc.y()) ** 2)
                    if dis < mindis:
                        mindis = dis
                        closest = yc
                if closest is not None:
                    self.bezierPoints.append((el + closest) / 2)

        self.make_controlPoints()
        return

    def make_controlPoints(self):
        self.bezier = []
        if len(self.bezierPoints) > 2:
            for i, m in enumerate(self.bezierPoints[:: self.STEP]):
                if self.STEP * i == len(self.bezierPoints) - 1:
                    next = self.bezierPoints[0] * self.is_closed + m * (
                        not self.is_closed
                    )
                else:
                    next = self.bezierPoints[self.STEP * i + 1]

                if i == 0:
                    prev = self.bezierPoints[-1] * self.is_closed + m * (
                        not self.is_closed
                    )
                else:
                    prev = self.bezierPoints[self.STEP * i - 1]

                rico = next - prev
                ricoN = rico / math.sqrt(rico.x() ** 2 + rico.y() ** 2)

                l2 = next - m
                dist2 = math.sqrt(l2.x() ** 2 + l2.y() ** 2)

                l1 = m - prev
                dist1 = math.sqrt(l1.x() ** 2 + l1.y() ** 2)

                c2 = m + ricoN * self.BEZIER_CONTROL * dist2
                c1 = m - ricoN * self.BEZIER_CONTROL * dist1

                self.bezier.append(BezierPoint(c1, m, c2))
        return

    def get_bezier_curve_iterator(self) -> Iterator[Tuple[BezierPoint, BezierPoint]]:
        if len(self.bezier) == 0:
            return iter(())
        shifted_by_one = copy.deepcopy(self.bezier)
        first_item = shifted_by_one.pop(0)
        shifted_by_one += [first_item]

        if self.is_closed:
            return zip(self.bezier, shifted_by_one)
        else:
            return zip(self.bezier[:-1], shifted_by_one[:-1])

    def paint_bezier_path(self, painter):
        if self.bezier != []:
            pen = QtG.QPen(QtG.QColor(255, 0, 0))
            pen.setWidthF(2.0)
            painter.setPen(pen)

            path = QtG.QPainterPath()
            path.moveTo(self.coordinateToScreen(self.bezier[0].m))

            for current_point, next_point in self.get_bezier_curve_iterator():
                path.cubicTo(
                    self.coordinateToScreen(current_point.c2),
                    self.coordinateToScreen(next_point.c1),
                    self.coordinateToScreen(next_point.m),
                )

                painter.drawPath(path)
        return

    def draw_grid(self, painter):
        painter.setPen(QtG.QPen(QtG.QColor(0, 0, 0)))
        vertical_line_count = int(
            self.width() / self.pixels_per_km / self.zoom_level / self.RASTER_WIDTH + 1
        )
        horizontal_line_count = int(
            self.height() / self.pixels_per_km / self.zoom_level / self.RASTER_WIDTH + 1
        )

        px_width = self.pixels_per_km * self.zoom_level * self.RASTER_WIDTH

        painter.drawLine(
            QtC.QPointF(self.width() // 2, 0),
            QtC.QPointF(self.width() // 2, self.height()),
        )
        for line_index in range((vertical_line_count + 1) // 2):
            painter.drawLine(
                QtC.QPointF(self.width() // 2 + line_index * px_width, 0),
                QtC.QPointF(self.width() // 2 + line_index * px_width, self.height()),
            )
            painter.drawLine(
                QtC.QPointF(self.width() // 2 - line_index * px_width, 0),
                QtC.QPointF(self.width() // 2 - line_index * px_width, self.height()),
            )

        painter.drawLine(
            QtC.QPointF(0, self.height() // 2),
            QtC.QPointF(self.width(), self.height() // 2),
        )
        for line_index in range((horizontal_line_count + 1) // 2):
            painter.drawLine(
                QtC.QPointF(0, self.height() // 2 + line_index * px_width),
                QtC.QPointF(self.width(), self.height() // 2 + line_index * px_width),
            )
            painter.drawLine(
                QtC.QPointF(0, self.height() // 2 - line_index * px_width),
                QtC.QPointF(self.width(), self.height() // 2 - line_index * px_width),
            )

    # Override the wheelEvent method to handle scrolling events
    def wheelEvent(self, event):
        if event.angleDelta().y() != 0:
            # Determine the direction of the scroll (up or down)
            scroll_direction = event.angleDelta().y() / abs(event.angleDelta().y())
            # distance between wheel event and the offset
            s = self.screenToCoordinate(event.pos())
            r = self.offset - s
            # Adjust the zoom level based on the scroll direction
            if scroll_direction > 0:
                if self.zoom_level < self.MAX_ZOOM:
                    self.zoom_level *= self.ZOOM
                    self.offset = s + r / self.ZOOM
            else:
                if self.zoom_level > self.MIN_ZOOM:
                    self.zoom_level /= self.ZOOM
                    self.offset = s + r * self.ZOOM
            self.update()

    def coordinateToScreen(self, coordinate):
        # Calculate the screen position of a given coordinate, taking zoom level and scroll position into account
        x = (
            coordinate.x() - self.offset.x()
        ) * self.pixels_per_km * self.zoom_level + self.rect().width() / 2
        y = (
            -(coordinate.y() - self.offset.y()) * self.pixels_per_km * self.zoom_level
            + self.rect().height() / 2
        )
        return QtC.QPoint(int(x), int(y))

    def screenToCoordinate(self, screen_pos):
        # Calculate the coordinate of a given screen position, taking zoom level and scroll position into account
        x = (screen_pos.x() - self.rect().width() / 2) / (
            self.pixels_per_km * self.zoom_level
        ) + self.offset.x()
        y = (
            -(screen_pos.y() - self.rect().height() / 2)
            / (self.pixels_per_km * self.zoom_level)
            + self.offset.y()
        )
        return QtC.QPointF(x, y)

    def keyPressEvent(self, event: QtG.QKeyEvent):
        if event.modifiers() == QtC.Qt.ControlModifier and event.key() == QtC.Qt.Key_S:
            self.save_track_layout()

    def save_track_layout(self):
        def get_track_name() -> str:
            time_string = datetime.datetime.now().strftime("%d%b_%H%M").lower()
            if self.LAYOUT_TYPE == "json":
                return f"track_{time_string}.json"
            else:
                return f"track_{time_string}.yaml"

        if self.LAYOUT_TYPE == "json":
            track_dict = self.as_dict()
        else:
            track_dict = self.create_yaml()
        file_path = (
            pathlib.Path.home()
            / f"autonomous2023/ROS/src/slam/slam_simulator/maps/{get_track_name()}"
        )
        with open(file_path, "w") as f:
            if self.LAYOUT_TYPE == "json":
                json.dump(track_dict, f)
            else:
                yaml.dump(track_dict, f)

    def as_dict(self):
        def get_track_limits() -> Tuple[QtC.QPointF, QtC.QPointF]:
            x_min = float("inf")
            x_max = -float("inf")

            y_min = float("inf")
            y_max = -float("inf")

            for cone in self.blue_cones + self.yellow_cones:
                if cone.x() < x_min:
                    x_min = cone.x()
                if cone.x() > x_max:
                    x_max = cone.x()

                if cone.y() < y_min:
                    y_min = cone.y()
                if cone.y() > y_max:
                    y_max = cone.y()

            return QtC.QPointF(x_min, y_min), QtC.QPointF(x_max, y_max)

        min_point, max_point = get_track_limits()
        width = max_point.x() - min_point.x()
        height = max_point.y() - min_point.y()
        self.car_rot %= 2 * math.pi
        if self.car_rot > math.pi:
            self.car_rot -= 2 * math.pi

        def cone_list_to_json_list(
            cone_list: List[QtC.QPointF], offset: QtC.QPointF = QtC.QPointF(0, 0)
        ) -> List[Dict]:
            json_list = []
            for cone in cone_list:
                offset_cone = cone + offset
                json_list.append({"pos": [offset_cone.x(), offset_cone.y()]})
            return json_list

        track_dict = {
            "parameters": {
                "is_closed": self.is_closed,
                "track_x_size": width,
                "track_y_size": height,
                "startpos_x": self.car_pos.x() - min_point.x(),
                "startpos_y": self.car_pos.y() - min_point.y(),
                "startrot": self.car_rot,
            },
            "middle_line": [
                bezier_point.to_dict(offset=min_point * -1)
                for bezier_point in self.bezier
            ],
            "cones": {
                "yellow": cone_list_to_json_list(
                    self.yellow_cones, offset=min_point * -1
                ),
                "blue": cone_list_to_json_list(self.blue_cones, offset=min_point * -1),
                "orange": cone_list_to_json_list(
                    self.orange_cones, offset=min_point * -1
                ),
            },
        }
        return track_dict

    def create_yaml(self):
        # Create a list to store all cones
        all_cones = []

        # Iterate over blue cone locations and create YAML entries
        for cone in self.blue_cones:
            cone_entry = {
                "covariance": [0.0] * 9,
                "observation": {
                    "belief": 1,
                    "location": {"x": cone.x(), "y": cone.y(), "z": 0},
                    "observation_class": 0,
                },
            }
            all_cones.append(cone_entry)

        # Iterate over yellow cone locations and create YAML entries
        for cone in self.yellow_cones:
            cone_entry = {
                "covariance": [0.0] * 9,
                "observation": {
                    "belief": 1,
                    "location": {"x": cone.x(), "y": cone.y(), "z": 0},
                    "observation_class": 1,
                },
            }
            all_cones.append(cone_entry)

        # Iterate over orange cone locations and create YAML entries
        for cone in self.orange_cones:
            cone_entry = {
                "covariance": [0.0] * 9,
                "observation": {
                    "belief": 1,
                    "location": {"x": cone.x(), "y": cone.y(), "z": 0},
                    "observation_class": 2,
                },
            }
            all_cones.append(cone_entry)

        # Create the final YAML data structure
        data = {
            "header": {
                "frame_id": "ugr/map",
                "seq": 0,
                "stamp": {"nsecs": 0, "secs": 0},
            },
            "observations": all_cones,
        }
        return data


class MainWindow(QtW.QMainWindow):
    def __init__(self, publisher, frame, trackfile_name=None):
        super().__init__(None)
        if trackfile_name is not None:
            layout_path = (
                pathlib.Path.home()
                / f"autonomous2023/ROS/src/slam/slam_simulator/maps/{trackfile_name}"
            )
            with open(layout_path, "r") as f:
                if trackfile_name.endswith(".json"):
                    dictio = json.load(f)
                    yellow_cones = dictio["cones"]["yellow"]
                    yellows = [
                        QtC.QPointF(c["pos"][0], c["pos"][1]) for c in yellow_cones
                    ]
                    blue_cones = dictio["cones"]["blue"]
                    blues = [QtC.QPointF(c["pos"][0], c["pos"][1]) for c in blue_cones]
                    orange_cones = dictio["cones"].get("orange")
                    if orange_cones is not None:
                        oranges = [
                            QtC.QPointF(c["pos"][0], c["pos"][1]) for c in orange_cones
                        ]
                    else:
                        oranges = []
                    is_closed = dictio["parameters"]["is_closed"]
                    startpos_x = dictio["parameters"]["startpos_x"]
                    startpos_y = dictio["parameters"]["startpos_y"]
                    startrot = dictio["parameters"]["startrot"]
                elif trackfile_name.endswith(".yaml"):
                    dictio = yaml.safe_load(f)
                    yellows = [
                        QtC.QPointF(
                            pose["observation"]["location"]["x"],
                            pose["observation"]["location"]["y"],
                        )
                        for pose in dictio["observations"]
                        if pose["observation"]["observation_class"] == 1
                    ]
                    blues = [
                        QtC.QPointF(
                            pose["observation"]["location"]["x"],
                            pose["observation"]["location"]["y"],
                        )
                        for pose in dictio["observations"]
                        if pose["observation"]["observation_class"] == 0
                    ]
                    oranges = [
                        QtC.QPointF(
                            pose["observation"]["location"]["x"],
                            pose["observation"]["location"]["y"],
                        )
                        for pose in dictio["observations"]
                        if pose["observation"]["observation_class"] == 2
                    ]
                    is_closed = False
                    startpos_x = 0
                    startpos_y = 0
                    startrot = 0
                else:
                    raise ValueError(
                        "Invalid file format. Only JSON and YAML files are supported."
                    )
            self.map_widget = MapWidget(
                publisher,
                frame,
                is_closed,
                startpos_x,
                startpos_y,
                startrot,
                yellows,
                blues,
                oranges,
            )
        else:
            # Create a new instance of MapWidget
            self.map_widget = MapWidget(publisher, frame)
        # Add the MapWidget to the main window
        self.setCentralWidget(self.map_widget)
        self.resize(800, 600)
        self.setWindowTitle("Track editor")
