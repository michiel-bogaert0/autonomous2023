#!/usr/bin/env python3
import rospkg
import rospy
import yaml
from genpy.message import fill_message_args
from nav_msgs.msg import Path
from node_fixture.fixture import (
    DiagnosticArray,
    DiagnosticStatus,
    create_diagnostic_message,
)
from node_fixture.node_management import ManagedNode


class PathPublisher(ManagedNode):
    def __init__(self):
        super().__init__("path_publisher")
        self.spin()

    def doConfigure(self):
        self.override_time = rospy.get_param("~override_time", True)

        self.path_publisher = rospy.Publisher(
            "/output/path",
            Path,
            queue_size=1,
            latch=True,
        )

        # Diagnostics Publisher
        self.diagnostics_pub = super().AddPublisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

    def doActivate(self):
        self.path = rospkg.RosPack().get_path(
            rospy.get_param("~package_path")
        ) + rospy.get_param("~path")
        self.publish_path()

    def publish_path(self):
        ros_path = Path()
        try:
            # Try to parse YAML
            with open(self.path, "r") as file:
                path = yaml.safe_load(file)
                fill_message_args(ros_path, path)

                self.path_publisher.publish(ros_path)

        except Exception:
            rospy.logerr(
                f"Error publishing path. Make sure that the file '{self.path}' exists, is readable and is valid YAML!"
            )
            self.diagnostics_pub.publish(
                create_diagnostic_message(
                    level=DiagnosticStatus.ERROR,
                    name="[CTRL CTRL] Path Publisher Status",
                    message="Error publishing path.",
                )
            )


node = PathPublisher()
