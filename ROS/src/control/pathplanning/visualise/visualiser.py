#! /usr/bin/python3
import json
import pathlib
import sys

import numpy as np
import rospy
import yaml
from PyQt5 import QtCore as QtC
from PyQt5 import QtWidgets as QtW
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped, PathArray
from visualise_pathplanning import MapWidget


class Visualiser:
    def __init__(self) -> None:
        rospy.init_node("visualiser")

        # Publisher voor local_map
        self.publisher = rospy.Publisher(
            "/output/local_map", ObservationWithCovarianceArrayStamped, queue_size=10
        )

        # Handler voor path_received subscription
        self.subscriber = rospy.Subscriber(
            "/input/debug/all_poses", PathArray, self.handle_path_received
        )

        self.frame = rospy.get_param("~frame", "ugr/car_base_link")
        self.track_file = rospy.get_param("~layout", "")

        # Initialize and start Qt application
        app = QtW.QApplication(sys.argv)
        if len(self.track_file) > 0:
            # If a track file is specified
            self.widget = MainWindow(self.publisher, self.frame, self.track_file)
        else:
            self.widget = MainWindow(self.publisher, self.frame)
        self.widget.show()
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
        self.widget.map_widget.receive_path(all_paths)


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
                    is_closed = True
                    startpos_x = 0
                    startpos_y = 0
                    startrot = 0
                else:
                    raise ValueError(
                        "Invalid file format. Only JSON and YAML files are supported."
                    )
            # Create a new instance of MapWidget
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
