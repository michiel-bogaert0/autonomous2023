#!/usr/bin/env python3
import os

import rospy
import yaml
from genpy.message import fill_message_args
from nav_msgs.msg import Path
from node_fixture.fixture import (
    DiagnosticArray,
    DiagnosticStatus,
    create_diagnostic_message,
)


class PathPublisher:
    def __init__(self):
        rospy.init_node("control_path_publisher")

        self.path = rospy.get_param(
            "~path", f"{os.path.dirname(__file__)}/../paths/straight_L100.yaml"
        )
        self.override_time = rospy.get_param("~override_time", True)

        self.path_publisher = rospy.Publisher(
            "/output/path",
            Path,
            queue_size=1,
            latch=True,
        )

        # Diagnostics Publisher
        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        try:
            self.publish_path()
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

        rospy.spin()

    def publish_path(self):
        # Try to parse YAML
        ros_path = Path()
        with open(self.path, "r") as file:
            path = yaml.safe_load(file)
            fill_message_args(ros_path, path)

            self.path_publisher.publish(ros_path)


node = PathPublisher()
