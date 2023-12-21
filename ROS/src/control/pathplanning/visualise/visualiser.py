#! /usr/bin/python3
import json
import pathlib
import sys

import numpy as np
import rospy
import yaml
from node_fixture.node_management import configure_node, set_state_active
from PyQt5 import QtCore as QtC
from PyQt5 import QtWidgets as QtW
from ugr_msgs.msg import Boundaries, ObservationWithCovarianceArrayStamped, PathArray
from visualise_pathplanning import MapWidget
from visualization_msgs.msg import MarkerArray


class Visualiser:
    def __init__(self) -> None:
        rospy.init_node("visualiser")

        # Publisher voor local_map
        self.publisher = rospy.Publisher(
            "/output/local_map", ObservationWithCovarianceArrayStamped, queue_size=10
        )

        # Handler voor path_received subscription
        self.path_subscriber = rospy.Subscriber(
            "/input/debug/all_poses", PathArray, self.handle_path_received
        )

        self.boundaries_subscriber = rospy.Subscriber(
            "/input/boundaries", Boundaries, self.handle_boundaries_received
        )

        self.points_subscriber = rospy.Subscriber(
            "/input/debug/markers", MarkerArray, self.handle_markers_received
        )

        self.frame = rospy.get_param("~frame", "ugr/car_base_link")
        self.track_file = rospy.get_param("~layout", "")
        configure_node("pathplanning")
        set_state_active("pathplanning")
        configure_node("boundary_estimation")
        set_state_active("boundary_estimation")
        # Initialize and start Qt application
        app = QtW.QApplication(sys.argv)
        if len(self.track_file) > 0:
            # If a track file is specified
            self.window = MainWindow(self.publisher, self.frame, self.track_file)
        else:
            self.window = MainWindow(self.publisher, self.frame)
        self.window.show()
        sys.exit(app.exec_())

    def handle_path_received(self, paths: PathArray):
        """
        Handles paths received from pathplanning

        Args:
            paths: the paths received
        """
        all_paths = []
        for path in paths.paths:
            all_paths.append(
                np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
            )
        self.window.map_widget.receive_path(all_paths)

    def handle_boundaries_received(self, boundaries: Boundaries):
        relBlue = np.array(
            [
                [p.pose.position.x, p.pose.position.y]
                for p in boundaries.left_boundary.poses
            ]
        )
        relYellow = np.array(
            [
                [p.pose.position.x, p.pose.position.y]
                for p in boundaries.right_boundary.poses
            ]
        )
        self.window.map_widget.receive_boundaries(relBlue, relYellow)

    def handle_markers_received(self, markers: MarkerArray):
        relCenterPoints = np.array(
            [
                [marker.pose.position.x, marker.pose.position.y]
                for marker in markers.markers
                if marker.ns == "pathplanning_vis/center_points"
            ]
        )
        relBadPoints = np.array(
            [
                [marker.pose.position.x, marker.pose.position.y]
                for marker in markers.markers
                if marker.ns == "pathplanning_vis/bad_points"
            ]
        )
        if len(relCenterPoints) > 0:
            self.window.map_widget.receive_centerPoints(relCenterPoints)
        if len(relBadPoints) > 0:
            self.window.map_widget.receive_badPoints(relBadPoints)


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
                    startpos_x = 0
                    startpos_y = 0
                    startrot = 0
                    is_closed = True
                    place_cones = False
                else:
                    raise ValueError(
                        "Invalid file format. Only JSON and YAML files are supported."
                    )
            # Create a new instance of MapWidget
            self.map_widget = MapWidget(
                publisher,
                frame,
                startpos_x,
                startpos_y,
                startrot,
                yellows,
                blues,
                oranges,
                is_closed,
                place_cones,
            )
        else:
            # Create a new instance of MapWidget
            self.map_widget = MapWidget(publisher, frame)
        # Add the MapWidget to the main window
        self.setCentralWidget(self.map_widget)
        # set window size when minimalized
        self.resize(800, 600)
        self.setWindowTitle("Track Tool")
        # Show the main window in full screen
        self.showMaximized()


if __name__ == "__main__":
    try:
        node = Visualiser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
