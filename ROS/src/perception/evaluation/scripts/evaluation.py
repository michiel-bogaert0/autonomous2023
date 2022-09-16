#! /usr/bin/python3


import rospy
from sensor_msgs.msg import NavSatFix
from ugr_msgs.msg import ConeLocation


class ConeLogger:
    """
    class that logs position of cones
    """

    def __init__(self):
        rospy.init_node("conelogger")
        self.type = "n"

        # keep track of a sequence of gps messages
        self.cone_position = []

        # only the message with the least covariance will be published
        self.covariances = []

        # use different gps's for blue and yellow cones
        self.subscriber_left = rospy.Subscriber(
            "/input/gps/left", NavSatFix, self.record_blue_cone
        )
        self.subscriber_right = rospy.Subscriber(
            "/input/gps/right", NavSatFix, self.record_yellow_cone
        )

        self.cone_pusher = rospy.Publisher("/output/cones", ConeLocation, queue_size=10)

    def handle_input(self) -> None:
        """
        Main function
        """
        while not rospy.is_shutdown():
            # take input from user (y = yellow cone, b = blue cone, ol = orange cone left, or = orange cone right )
            cone_type = input("insert type: ")
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
        Fills the list with gps messages coming from the left gps
        args:
            gps_msg: message comming from the left ublox gps
        """
        if self.type == "y" or self.type == "ol":
            self.cone_position.append(gps_msg)
            self.covariances.append(
                gps_msg.position_covariance[0]
                + gps_msg.position_covariance[4]
                + gps_msg.position_covariance[8]
            )

    def record_blue_cone(self, gps_msg) -> None:
        """
        Fills the list with gps messages coming from the right gps
        args:
            gps_msg: message comming from the right ublox gps
        """
        if self.type == "b" or self.type == "or":
            self.cone_position.append(gps_msg)
            self.covariances.append(
                gps_msg.position_covariance[0]
                + gps_msg.position_covariance[4]
                + gps_msg.position_covariance[8]
            )

    def push_cone_location(self, cone_type: str) -> None:
        """
        given the recorded cone positions, publish the gps coordinates with minimal covariance, include the cone type
        args:
            cone_type: color of the cone (options are b(lue), y(ellow), or (orange right) or ol (orange left))
        """
        # get the cone location with the least covariance
        if cone_type == "y":
            cone_type_int = 1
        elif cone_type == "b":
            cone_type_int = 0
        elif "o" in cone_type:
            cone_type_int = 2

        rospy.loginfo(len(self.cone_position))
        most_accurate = self.covariances.index(min(self.covariances))
        self.cone_pusher.publish(
            ConeLocation(
                location=self.cone_position[most_accurate], cone_type=cone_type_int
            )
        )


if __name__ == "__main__":
    node = ConeLogger()
    node.handle_input()
