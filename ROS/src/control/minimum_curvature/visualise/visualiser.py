#! /usr/bin/python3
import json
import pathlib
import sys

import numpy as np
import rospy
import yaml
from nav_msgs.msg import Path
from node_fixture.node_manager import configure_node, set_state_active
from PyQt5 import QtCore as QtC
from PyQt5 import QtWidgets as QtW
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped
from visualise_mincurv import MapWidget


class Visualiser:
    def __init__(self) -> None:
        rospy.init_node("visualiser_mincurv")

        # Publisher voor local_map
        self.map_publisher = rospy.Publisher(
            "/output/local_map", ObservationWithCovarianceArrayStamped, queue_size=10
        )

        self.frame = rospy.get_param("~frame", "ugr/car_base_link")
        self.track_file = rospy.get_param("~layout", "")
        rospy.wait_for_service("/node_managing/minimum_curvature/get", timeout=1.0)

        configure_node("pathplanning")
        configure_node("minimum_curvature")
        set_state_active("pathplanning")
        # set_state_active("minimum_curvature")

        # Subscriber voor minimum curvature path
        self.mincurvpath_subscriber = rospy.Subscriber(
            "/input/path", Path, self.handle_path_received
        )

        # Subscriber for extra path
        self.extrapath_subscriber = rospy.Subscriber(
            "/input/path_extra", Path, self.handle_path_received_extra
        )

        # Subscriber for iqp path
        self.iqppath_subscriber = rospy.Subscriber(
            "/input/path_iqp", Path, self.handle_path_received_iqp
        )

        # Subscriber for reference path
        self.refpath_subscriber = rospy.Subscriber(
            "/input/path_ref", Path, self.handle_path_received_ref
        )

        # Initialize and start Qt application
        app = QtW.QApplication(sys.argv)
        if len(self.track_file) > 0:
            # If a track file is specified
            self.window = MainWindow(self.map_publisher, self.frame, self.track_file)
        else:
            self.window = MainWindow(self.map_publisher, self.frame)
        self.window.show()
        sys.exit(app.exec_())

    def handle_path_received(self, path: Path):
        """
        Handles the path received from the minimum curvature node

        Args:
            path (Path): The path received from the minimum curvature node
        """
        self.window.map_widget.receive_path(
            np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        )

    def handle_path_received_extra(self, path: Path):
        """
        Handles the extra path received from the minimum curvature node

        Args:
            path (Path): The extra path received from the minimum curvature node
        """
        self.window.map_widget.receive_path_extra(
            np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        )

    def handle_path_received_iqp(self, path: Path):
        """
        Handles the iqp path received from the minimum curvature node

        Args:
            path (Path): The iqp path received from the minimum curvature node
        """
        self.window.map_widget.receive_path_iqp(
            np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        )

    def handle_path_received_ref(self, path: Path):
        """
        Handles the reference path received from the minimum curvature node

        Args:
            path (Path): The reference path received from the minimum curvature node
        """
        self.window.map_widget.receive_path_ref(
            np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        )


class MainWindow(QtW.QMainWindow):
    def __init__(self, map_publisher, frame, trackfile_name=None):
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
                else:
                    raise ValueError(
                        "Invalid file format. Only JSON and YAML files are supported."
                    )
            # Create a new instance of MapWidget
            self.map_widget = MapWidget(
                map_publisher,
                frame,
                startpos_x,
                startpos_y,
                startrot,
                yellows,
                blues,
                oranges,
            )
        else:
            # Create a new instance of MapWidget
            self.map_widget = MapWidget(map_publisher, frame)
        # Add the MapWidget to the main window
        self.setCentralWidget(self.map_widget)
        # set window size when minimalized
        self.resize(800, 600)
        self.setWindowTitle("Visualiser minimum curvature")
        # Show the main window in full screen
        self.showMaximized()


if __name__ == "__main__":
    try:
        node = Visualiser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
