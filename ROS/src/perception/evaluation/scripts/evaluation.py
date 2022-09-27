#! /usr/bin/python3

import rospy
from sensor_msgs.msg import NavSatFix
from ugr_msgs.msg import ConeLocation
from collections import deque


class ConeLogger:
    """
    class that logs position of cones
    """

    def __init__(self):
        rospy.init_node("cone_logger")
        self.type = "n"

        # keep track of a sequence of gps messages
        self.cone_position = deque([], maxlen=30)

        # only the message with the least covariance will be published
        self.covariances = deque([], maxlen=30)

        # use different gps's for blue and yellow cones
        self.subscriber_left = rospy.Subscriber(
            "/input/gps/left", NavSatFix, self.record_blue_cone
        )
        self.subscriber_right = rospy.Subscriber(
            "/input/gps/right", NavSatFix, self.record_yellow_cone
        )

        self.cone_pusher = rospy.Publisher("/output/cones", ConeLocation, queue_size=10)

        self.cone_colors = {"b": 0, "y": 1, "or": 2, "ol": 2}

    def handle_input(self) -> None:
        """
        Main function
        """
        while not rospy.is_shutdown():
            # take input from user (y = yellow cone, b = blue cone, ol = orange cone left, or = orange cone right )
            cone_type = input("insert type: ")
            if cone_type not in self.cone_colors:
                rospy.logwarn("bad input type")
                continue
            self.type = cone_type
            # sleep for three seconds -> the node will fill the cone_position and covariances list during that period
            rospy.sleep(3)
            # stop filling the lists
            self.type = "n"
            # push the correct cone location
            self.push_cone_location(cone_type)
            # reset lists
            self.cone_position = []
            self.covariances = []

    def record_yellow_cone(self, gps_msg: NavSatFix) -> None:
        """
        Fills the list with gps messages coming from the right gps
        args:
            gps_msg: message coming from the right ublox gps
        """
        if self.type == "y" or self.type == "ol":
            self.cone_position.append(gps_msg)
            self.covariances.append(sum(gps_msg.position_covariance))

    def record_blue_cone(self, gps_msg: NavSatFix) -> None:
        """
        Fills the list with gps messages coming from the left gps
        args:
            gps_msg: message coming from the left ublox gps
        """
        if self.type == "b" or self.type == "or":
            self.cone_position.append(gps_msg)
            self.covariances.append(sum(gps_msg.position_covariance))

    def push_cone_location(self, cone_type: str) -> None:
        """
        given the recorded cone positions, publish the gps coordinates with minimal covariance, include the cone type
        args:
            cone_type: color of the cone (options are b(lue), y(ellow), or (orange right) or ol (orange left))
        """
        # get the cone location with the least covariance
        cone_type_int = self.cone_colors[cone_type]

        try:

            most_accurate = self.covariances.index(min(self.covariances))
            self.cone_pusher.publish(
                ConeLocation(
                    location=self.cone_position[most_accurate], cone_type=cone_type_int
                )
            )
        except:
            rospy.logerr("exception while trying to publish a cone location!")


if __name__ == "__main__":
    node = ConeLogger()
    node.handle_input()
