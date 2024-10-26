#!/usr/bin/env python3
import math
import os
import sys

import rospy
import yaml
from genpy.message import fill_message_args
from node_fixture.fixture import (
    DiagnosticArray,
    DiagnosticStatus,
    create_diagnostic_message,
)
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped

sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "trackgen"))
)
from track_generator import TrackGenerator  # noqa: E402


class MapPublisher:
    def __init__(self):
        rospy.init_node("slam_simulator_map_publisher")

        self.generate_track = rospy.get_param("~generate_track")
        self.length = rospy.get_param("~track_length")

        if self.generate_track:
            self.map = rospy.get_param("~gen_map")
            gen_cfg = {
                "min_corner_radius": 3,
                "max_frequency": 7,
                "amplitude": 1 / 3,
                "check_self_intersection": True,
                "starting_amplitude": 0.4,
                "rel_accuracy": 0.005,
                "margin": 0,
                "starting_straight_length": 0,
                "starting_straight_downsample": 2,
                "min_cone_spacing": 3 * math.pi / 16,
                "max_cone_spacing": 5,
                "track_width": 3,
                "cone_spacing_bias": 0.5,
                "starting_cone_spacing": 0.5,
                "length": self.length,
            }
            gen = TrackGenerator(gen_cfg)  # generate track
            start_cones, left_cones, right_cones = gen()

            # Save track
            TrackGenerator.write_to_yaml(
                self.map,
                start_cones,
                left_cones,
                right_cones,
                overwrite=True,
            )
        else:
            self.map = rospy.get_param(
                "~map", f"{os.path.dirname(__file__)}/../maps/circle_R15.yaml"
            )
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

        rospy.spin()

    def publish_map(self):
        # Try to parse YAML
        ros_map = ObservationWithCovarianceArrayStamped()
        with open(self.map, "r") as file:
            map = yaml.safe_load(file)
            fill_message_args(ros_map, map)

            self.map_publisher.publish(ros_map)


node = MapPublisher()
