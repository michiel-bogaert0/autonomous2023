#!/usr/bin/env python3
import yaml
import rospy
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped

from genpy.message import fill_message_args


class MapPublisher():
    def __init__(self):

        rospy.init_node("slam_simulator_map_publisher")

        self.map = rospy.get_param(
            "~map", "/home/lowiek/autonomous2023/ROS/src/slam/slam_simulator/maps/rc_gt1.yaml")
        self.override_time = rospy.get_param("~override_time", True)

        self.map_publisher = rospy.Publisher(
            "/output/map",
            ObservationWithCovarianceArrayStamped,
            queue_size=1,
            latch=True,
        )

        try:
            self.publish_map()
        except:
            rospy.logerr(
                f"Error publishing map. Make sure that the file '{self.map}' exists, is readable and is valid YAML!")

        rospy.spin()

    def publish_map(self):

        # Try to parse YAML
        ros_map = ObservationWithCovarianceArrayStamped()
        with open(self.map, 'r') as file:
            map = yaml.safe_load(file)
            fill_message_args(ros_map, map)
        
            self.map_publisher.publish(ros_map)


node = MapPublisher()
