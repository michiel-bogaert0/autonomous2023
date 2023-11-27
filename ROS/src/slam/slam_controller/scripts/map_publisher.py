#!/usr/bin/env python3
import rospkg
import rospy
import yaml
from genpy.message import fill_message_args
from node_fixture.fixture import (
    DiagnosticArray,
    DiagnosticStatus,
    create_diagnostic_message,
)
from node_fixture.node_management import ManagedNode
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped


class MapPublisher(ManagedNode):
    def __init__(self):
        super().__init__("map_publisher")
        rospy.init_node("slam_map_publisher")

        rospy.spin()

    def doConfigure(self):
        self.mappkg = rospy.get_param("~map_pkg", "slam_simulator")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(self.mappkg)
        map_path = rospy.get_param("~map", "/maps/circle_R15.yaml")
        self.map = pkg_path + map_path

        self.override_time = rospy.get_param("~override_time", True)

        self.map_publisher = rospy.Publisher(
            "/output/map",
            ObservationWithCovarianceArrayStamped,
            queue_size=1,
            latch=True,
        )

        # Diagnostics Publisher
        self.diagnostics = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

    def doActivate(self):
        try:
            self.publish_map()
        except Exception:
            rospy.logerr(
                f"Error publishing map. Make sure that the file '{self.map}' exists, is readable and is valid YAML!"
            )
            self.diagnostics.publish(
                create_diagnostic_message(
                    level=DiagnosticStatus.ERROR,
                    name="[SLAM SIM] Map Publisher Status",
                    message="Error publishing map.",
                )
            )

    def publish_map(self):
        # Try to parse YAML
        ros_map = ObservationWithCovarianceArrayStamped()
        with open(self.map, "r") as file:
            map = yaml.safe_load(file)
            fill_message_args(ros_map, map)

            self.map_publisher.publish(ros_map)


node = MapPublisher()
