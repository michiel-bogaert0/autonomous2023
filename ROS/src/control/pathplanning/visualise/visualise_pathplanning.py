import datetime
import json
import math
import pathlib
from typing import Dict, List, Optional, Tuple

import numpy as np
import rospy
import yaml
from bezier import make_bezier, make_middelline
from buttons import Buttons
from draw import Draw
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from PyQt5 import QtCore as QtC
from PyQt5 import QtGui as QtG
from PyQt5 import QtWidgets as QtW
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)
from utils import car_to_real_transform, dist, get_local_poses, real_to_car_transform


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
        gt_path_publisher,
        frame,
        startpos_x=0,
        startpos_y=0,
        startrot=0,
        yellows=None,
        blues=None,
        oranges=None,
        closed=False,
        place_cones=True,
    ):
        super().__init__(None)
        self.initWidget()

        self.buttons = Buttons(self)
        self.draw = Draw(self)

        # publisher used to publish observation messages
        self.map_publisher = map_publisher
        # publisher used to publish observation messages
        self.gt_path_publisher = gt_path_publisher
        # frameID used to publish observation messages
        self.frame = frame

        # initialize paths
        self.path = None
        self.pathnr = -1
        self.nr_paths = 0
        self.all_paths = []

        # initialize smoothed path
        self.smoothed_path = None

        # initialize boundaries
        self.blue_boundary = []
        self.yellow_boundary = []
        # markers for centerPoints and badPoints
        self.centerPoints = []
        self.badPoints = []

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
        self.buttons.select_all_clicked()

        self.is_closed = bool(closed)
        self.place_cones = bool(place_cones)
        self.middelline_on = False
        self.trackbounds_on = False
        self.debug_badPoints = True
        self.debug_centerPoints = True
        self.paths_on = True
        self.smoothed_path_on = True
        self.boundary_estimation_on = True
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
        self.middelPoints = []
        self.middel_bezier = []
        self.blue_bezier = []
        self.yellow_bezier = []

        self.update()
        self.publish_local_map()

    def publish_local_map(self):
        """
        Publishes local map of the selected cones

        """
        cones = get_local_poses(
            self.selected_blue_cones,
            self.selected_yellow_cones,
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

        self.map_publisher.publish(local)

    def publish_gt_path(self):
        gt_path_msg = Path()
        gt_path_msg.header.stamp = rospy.Time.now()
        gt_path_msg.header.frame_id = self.frame

        gt_path = real_to_car_transform(self.path, self.car_pos, self.car_rot)

        for cone in gt_path:
            pose = PoseStamped()
            pose.pose.position.x = cone[0]
            pose.pose.position.y = cone[1]
            pose.pose.position.z = 0

            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            pose.header = gt_path_msg.header
            gt_path_msg.poses.append(pose)

        self.gt_path_publisher.publish(gt_path_msg)

    def receive_path(self, rel_path: np.ndarray):
        self.smoothed_path = car_to_real_transform(rel_path, self.car_pos, self.car_rot)
        self.update()

    def receive_all_paths(self, rel_paths: List[np.ndarray]):
        self.nr_paths = len(rel_paths)
        self.pathnr = min(self.pathnr, self.nr_paths - 1)
        if self.nr_paths == 0:
            self.pathnr = -1
        else:
            self.pathnr = max(self.pathnr, 0)
        self.all_paths = [
            car_to_real_transform(rel_path, self.car_pos, self.car_rot)
            for rel_path in rel_paths
        ]
        if self.pathnr >= 0:
            self.path = self.all_paths[self.pathnr]
        self.update()

    def receive_boundaries(self, relBlue: np.ndarray, relYellow: np.ndarray):
        self.blue_boundary = car_to_real_transform(relBlue, self.car_pos, self.car_rot)
        self.yellow_boundary = car_to_real_transform(
            relYellow, self.car_pos, self.car_rot
        )
        self.update()

    def receive_centerPoints(self, centerPoints: np.ndarray):
        self.centerPoints = car_to_real_transform(
            centerPoints, self.car_pos, self.car_rot
        )
        self.update()

    def receive_badPoints(self, badPoints: np.ndarray):
        self.badPoints = car_to_real_transform(badPoints, self.car_pos, self.car_rot)
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
        selected_cone = self.get_selected_element(event)
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
                self.empty_pathplanning_input()
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
                self.empty_pathplanning_input()
                self.update()

            # Drag a cone or car_handle
            else:
                self.selection = selected_cone
                self.selected_location = event.pos()
                self.drag_start_pos = event.pos()
        # Drag the screen
        elif event.modifiers() & QtC.Qt.ControlModifier or not self.place_cones:
            self.drag_map = True
            self.drag_start_pos = event.pos()
        elif self.place_cones:
            # orange cones with alt key
            if event.modifiers() == QtC.Qt.AltModifier:
                self.orange_cones.append(self.screenToCoordinate(event.pos()))
            # Place a yellow cone
            elif event.button() == QtC.Qt.LeftButton and not (
                event.modifiers() & QtC.Qt.ShiftModifier
            ):
                # Add a visual point to the list at the position where the user clicked
                point = self.screenToCoordinate(event.pos())
                self.yellow_cones.append(point)
                self.selected_yellow_cones.append(point)
                # Trigger a repaint of the MapWidget to update the visual points
            # Place a blue cone
            elif event.button() == QtC.Qt.RightButton and not (
                event.modifiers() & QtC.Qt.ShiftModifier
            ):
                # Add a visual point to the list at the position where the user clicked
                point = self.screenToCoordinate(event.pos())
                self.blue_cones.append(point)
                self.selected_blue_cones.append(point)
                # Trigger a repaint of the MapWidget to update the visual points
            self.empty_pathplanning_input()
            self.update()

    def mouseDoubleClickEvent(self, event):
        if event.button() == QtC.Qt.LeftButton:
            selected_cone = self.get_selected_element(event)
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
                self.empty_pathplanning_input()
                self.update()

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
                self.selection += (
                    drag_distance  # QtC.QPointF(drag_distance)/(self.zoom_level)
                )
                self.drag_start_pos = event.pos()
            self.update()

        # If the Control key is pressed and the mouse is dragged, move the map
        elif self.drag_map and self.drag_start_pos is not None:
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

        # the only time you're sure that you don't get a transformation fault in the path
        self.empty_pathplanning_input()
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
            self.empty_pathplanning_input()
            self.update()

    def keyPressEvent(self, event: QtG.QKeyEvent):
        if event.modifiers() == QtC.Qt.ControlModifier and event.key() == QtC.Qt.Key_S:
            self.save_track_layout()
        if event.key() == QtC.Qt.Key_Return:
            self.publish_local_map()
        else:
            # if a number is pressed
            if event.key() == QtC.Qt.Key_0:
                self.pathnr = 0
            elif event.key() == QtC.Qt.Key_1:
                self.pathnr = 0
            elif event.key() == QtC.Qt.Key_2:
                self.pathnr = 1
            elif event.key() == QtC.Qt.Key_3:
                self.pathnr = 2
            elif event.key() == QtC.Qt.Key_4:
                self.pathnr = 3
            elif event.key() == QtC.Qt.Key_5:
                self.pathnr = 4
            elif event.key() == QtC.Qt.Key_6:
                self.pathnr = 5
            elif event.key() == QtC.Qt.Key_7:
                self.pathnr = 6
            elif event.key() == QtC.Qt.Key_8:
                self.pathnr = 7
            elif event.key() == QtC.Qt.Key_9:
                self.pathnr = 8
            elif event.key() == QtC.Qt.Key_Plus:
                self.pathnr += 1
            elif event.key() == QtC.Qt.Key_Minus:
                self.pathnr -= 1
            self.pathnr = min(self.pathnr, self.nr_paths - 1)
            if self.nr_paths == 0:
                self.pathnr = -1
            else:
                self.pathnr = max(self.pathnr, 0)

            if self.pathnr >= 0:
                self.path = self.all_paths[self.pathnr]
            self.update()

    def empty_pathplanning_input(self):
        self.all_paths = []
        self.path = None
        self.centerPoints = []
        self.badPoints = []

    def update_car(self):
        self.car_rot_handle: QtC.QPointF = self.car_pos + (
            self.CAR_POINT_SIZE / 2
        ) * QtC.QPointF(math.cos(self.car_rot), math.sin(self.car_rot))
        return None

    # Override the paintEvent method to draw the visual points
    def paintEvent(self, event):
        painter = QtG.QPainter(self)
        self.draw.draw_grid(painter)

        if self.middelline_on:
            self.middelPoints = make_middelline(self.blue_cones, self.yellow_cones)
            self.middel_bezier = make_bezier(self.middelPoints, self.is_closed)
            self.draw.draw_bezier_line(
                self.middel_bezier, painter, QtG.QColor(0, 0, 0, 70)
            )
            self.draw.draw_points(self.middelPoints, painter, QtG.QColor(0, 0, 0, 70))
            self.publish_gt_path()

        if self.trackbounds_on:
            self.blue_bezier = make_bezier(self.blue_cones, self.is_closed)
            self.draw.draw_bezier_line(
                self.blue_bezier, painter, QtG.QColor(QtC.Qt.blue)
            )

            self.yellow_bezier = make_bezier(self.yellow_cones, self.is_closed)
            self.draw.draw_bezier_line(
                self.yellow_bezier, painter, QtG.QColor(QtC.Qt.yellow)
            )

        self.draw.draw_cones(painter)

        if self.debug_centerPoints:
            self.draw.draw_points(self.centerPoints, painter, QtG.QColor(255, 215, 0))
        if self.debug_badPoints:
            self.draw.draw_points(self.badPoints, painter, QtG.QColor(255, 0, 255))

        if self.boundary_estimation_on:
            self.draw.draw_line(
                self.blue_boundary,
                painter,
                QtG.QColor(QtC.Qt.blue),
            )
            self.draw.draw_line(
                self.yellow_boundary,
                painter,
                QtG.QColor(QtC.Qt.yellow),
            )
        if self.paths_on:
            self.draw.draw_line(
                self.path, painter, QtG.QColor(QtC.Qt.green), QtG.QColor(QtC.Qt.red)
            )
        if self.smoothed_path_on:
            self.draw.draw_line(self.smoothed_path, painter, QtG.QColor(QtC.Qt.red))
        self.draw.draw_car(painter)
        self.draw.draw_scale(painter)
        if self.paths_on:
            self.draw.draw_pathnr(painter)

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
                "is_closed": self.is_closed,
                "track_x_size": width,
                "track_y_size": height,
                "startpos_x": self.car_pos.x() - min_point.x(),
                "startpos_y": self.car_pos.y() - min_point.y(),
                "startrot": self.car_rot,
            },
            "middle_line": [
                bezier_point.to_dict(offset=min_point * -1)
                for bezier_point in self.middel_bezier
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
