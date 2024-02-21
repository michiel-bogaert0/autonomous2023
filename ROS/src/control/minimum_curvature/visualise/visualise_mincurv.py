import datetime
import json
import math
import pathlib
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml
from bezier import make_bezier, make_refline
from buttons import Buttons
from draw import Draw
from node_fixture.node_manager import set_state_active, set_state_inactive
from PyQt5 import QtCore as QtC
from PyQt5 import QtGui as QtG
from PyQt5 import QtWidgets as QtW
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)
from utils import car_to_real_transform, dist, get_local_poses


class MapWidget(QtW.QFrame):
    # Constants
    ZOOM = 1.1
    MAX_ZOOM = 1000
    MIN_ZOOM = 10
    INIT_ZOOM = 50
    # sizes in meters
    CAR_POINT_SIZE = 0.8
    CAR_HANDLE_SIZE = 0.5
    CONE_SIZE = 0.4
    LAYOUT_TYPE = "yaml"

    def __init__(
        self,
        map_publisher,
        frame,
        startpos_x=0,
        startpos_y=0,
        startrot=0,
        yellows=None,
        blues=None,
        oranges=None,
    ):
        super().__init__(None)
        self.initWidget()

        self.buttons = Buttons(self)
        self.draw = Draw(self)

        # publisher used to publish observation messages
        self.map_publisher = map_publisher

        # frameID used to publish observation messages
        self.frame = frame

        # intialize minimum curvature path
        self.mincurv_path = None

        # initialize cone lists
        if yellows is None:
            yellows = []
        if blues is None:
            blues = []
        if oranges is None:
            oranges = []

        # Create a list to store the cones
        self.yellow_cones = yellows
        self.blue_cones = blues
        self.orange_cones = oranges
        self.selected_yellow_cones = []
        self.selected_blue_cones = []

        self.refline_on = True
        self.trackbounds_on = True
        self.cones_on = False
        self.mincurv_on = True
        self.compute_on = True
        self.buttons.cones_clicked()
        self.buttons.set_buttons()

        # currently selected element
        self.selection: Optional[QtC.QPoint] = None

        # Set the initial zoom level
        self.zoom_level = self.INIT_ZOOM

        # The offset between the center of the screen and (0,0) of the real_coordinates on the map
        self.offset = QtC.QPointF(0, 0)
        # Boolean True when dragging around the map (using ctrl + mouse drag)
        self.drag_map = False

        # Start position of the car
        self.car_pos: QtC.QPointF = QtC.QPointF(startpos_x, startpos_y)
        self.car_rot: float = startrot
        # position of the car rotation handle
        self.car_rot_handle: QtC.QPointF = self.car_pos + (
            self.CAR_POINT_SIZE / 2
        ) * QtC.QPointF(math.cos(self.car_rot), math.sin(self.car_rot))

        # initialize list of bezier points wich represent the middle of the track
        self.refPoints = []
        self.ref_bezier = []
        self.blue_bezier = []
        self.yellow_bezier = []

    def publish_local_map(self):
        """
        Publishes local map of the selected cones

        """
        cones = get_local_poses(
            self.blue_cones,
            self.yellow_cones,
            self.car_pos,
            self.car_rot,
        )

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
        if self.compute_on:
            self.map_publisher.publish(local)
            self.compute_on = False

            set_state_inactive("minimum_curvature")
            set_state_active("minimum_curvature")

    def receive_path(self, rel_path: np.ndarray):
        self.mincurv_path = car_to_real_transform(rel_path, self.car_pos, self.car_rot)
        self.buttons.computeButton.setStyleSheet("background-color: gray")

        self.update()

    def get_selected_element(self, event) -> Optional[QtC.QPoint]:
        # This is the position on the screen where the user clicked
        press_location = event.pos()

        if (
            dist(press_location, self.coordinateToScreen(self.car_rot_handle))
            < self.CAR_HANDLE_SIZE / 2 * self.zoom_level
        ):
            return self.car_rot_handle

        if (
            dist(press_location, self.coordinateToScreen(self.car_pos))
            < self.CAR_POINT_SIZE / 2 * self.zoom_level
        ):
            return self.car_pos

        selectables = self.orange_cones + self.blue_cones + self.yellow_cones

        for selectable in selectables:
            control_point_canvas = self.coordinateToScreen(selectable)

            if (
                dist(press_location, control_point_canvas)
                < self.CONE_SIZE / 2 * self.zoom_level
            ):
                return selectable

        return None

    # Override the mousePressEvent method to handle mouse click events
    def mousePressEvent(self, event):
        # Get the position of the click
        selected_cone = self.get_selected_element(event)
        if selected_cone is not None:
            self.selection = selected_cone
            self.selected_location = event.pos()
            self.drag_start_pos = event.pos()

        # Drag the screen
        else:
            self.drag_map = True
            self.drag_start_pos = event.pos()

    def mouseMoveEvent(self, event):
        # If the selection is not None, the user is dragging a point
        if self.selection is not None:
            drag_distance = self.screenToCoordinate(
                event.pos()
            ) - self.screenToCoordinate(self.drag_start_pos)
            self.selection += drag_distance
            self.drag_start_pos = event.pos()
            self.update()

        # If the Control key is pressed and the mouse is dragged, move the map
        if self.drag_map and self.drag_start_pos is not None:
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

    def mouseReleaseEvent(self, event):
        # Reset the flag and the drag start position
        self.selection = None
        self.drag_map = False
        self.drag_start_pos = None
        self.update_car()
        self.empty_mincurv_input()
        self.update()

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
            self.empty_mincurv_input()
            self.update()

    def keyPressEvent(self, event: QtG.QKeyEvent):
        if event.modifiers() == QtC.Qt.ControlModifier and event.key() == QtC.Qt.Key_S:
            self.save_track_layout()
        else:
            self.update()

    def empty_mincurv_input(self):
        self.path = None

    def update_car(self):
        self.car_rot_handle: QtC.QPointF = self.car_pos + (
            self.CAR_POINT_SIZE / 2
        ) * QtC.QPointF(math.cos(self.car_rot), math.sin(self.car_rot))
        return None

    # Override the paintEvent method to draw the visual points
    def paintEvent(self, event):
        self.publish_local_map()

        painter = QtG.QPainter(self)
        self.draw.draw_grid(painter)

        self.draw.draw_cones(painter)

        if self.refline_on:
            self.refPoints = make_refline(self.blue_cones, self.yellow_cones)
            self.ref_bezier = make_bezier(self.refPoints)
            self.draw.draw_bezier_line(
                self.ref_bezier, painter, QtG.QColor(0, 0, 0, 70)
            )
            self.draw.draw_points(self.refPoints, painter, QtG.QColor(0, 0, 0, 70))

        if self.trackbounds_on:
            self.blue_bezier = make_bezier(self.blue_cones)
            self.draw.draw_bezier_line(
                self.blue_bezier, painter, QtG.QColor(QtC.Qt.blue)
            )

            self.yellow_bezier = make_bezier(self.yellow_cones)
            self.draw.draw_bezier_line(
                self.yellow_bezier, painter, QtG.QColor(QtC.Qt.yellow)
            )

        if self.mincurv_on:
            self.draw.draw_line(
                self.mincurv_path,
                painter,
                QtG.QColor(QtC.Qt.green),
                QtG.QColor(QtC.Qt.red),
            )

        self.draw.draw_car(painter)
        self.draw.draw_scale(painter)

        painter.end()

    def coordinateToScreen(self, coordinate):
        # Calculate the screen position of a given coordinate, taking zoom level and scroll position into account
        x = (
            coordinate.x() - self.offset.x()
        ) * self.zoom_level + self.rect().width() / 2
        y = (
            -(coordinate.y() - self.offset.y()) * self.zoom_level
            + self.rect().height() / 2
        )
        return QtC.QPoint(int(x), int(y))

    def screenToCoordinate(self, screen_pos):
        # Calculate the coordinate of a given screen position, taking zoom level and scroll position into account
        x = (screen_pos.x() - self.rect().width() / 2) / (
            self.zoom_level
        ) + self.offset.x()
        y = (
            -(screen_pos.y() - self.rect().height() / 2) / (self.zoom_level)
            + self.offset.y()
        )
        return QtC.QPointF(x, y)

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
                "track_x_size": width,
                "track_y_size": height,
                "startpos_x": self.car_pos.x() - min_point.x(),
                "startpos_y": self.car_pos.y() - min_point.y(),
                "startrot": self.car_rot,
            },
            "ref_line": [
                bezier_point.to_dict(offset=min_point * -1)
                for bezier_point in self.ref_bezier
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

    def initWidget(self):
        self.setFocus()
        # Set the size policy of the MapWidget to Expanding
        self.setSizePolicy(QtW.QSizePolicy.Expanding, QtW.QSizePolicy.Expanding)
        # Set the background color of the MapWidget to white
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QtC.Qt.white)
        self.setPalette(p)
