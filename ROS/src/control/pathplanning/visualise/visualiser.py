#! /usr/bin/python3
import sys

import numpy as np
import rospy
from PyQt5 import QtWidgets as QtW
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped, PathArray
from visualise_pathplanning import MainWindow


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

        app = QtW.QApplication(sys.argv)

        self.frame = rospy.get_param("~frame", "ugr/car_base_link")
        self.track_file = rospy.get_param("~layout", "")

        if len(self.track_file) > 0:
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


if __name__ == "__main__":
    try:
        node = Visualiser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
