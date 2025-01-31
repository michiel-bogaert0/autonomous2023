#! /usr/bin/python3

from functools import partial

import rospy
from slam_controller.srv import (
    GetMap,
    GetMapRequest,
    GetMapResponse,
    SetMap,
    SetMapRequest,
    SetMapResponse,
)
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped


class MapServer:
    def __init__(self) -> None:
        """
        Spins up a map server to keep maps (ugr_msgs/ObservationWithCovarianceArrayStamped)
        Also publishes it to the topic {prefix}/{namespace} with latching enabled
        """

        rospy.init_node("slam_controller_map_server")

        # Map dictionary
        self.maps = {}
        self.publishers = {}

        self.prefix = rospy.get_param("~topic_prefix", "/ugr/car/map")

        rospy.Service("ugr/srv/slam_map_server/set", SetMap, partial(self.set_map))
        rospy.Service("ugr/srv/slam_map_server/get", GetMap, partial(self.get_map))

        rospy.spin()

    def set_map(self, req: SetMapRequest):
        """
        Sets a map

        Args:
          req: the SetMap request, see SetMap.srv file
        """

        is_new_publisher = req.name not in self.maps.keys()
        self.maps[req.name] = req.map

        # Also publish
        publisher = (
            rospy.Publisher(
                f"/{self.prefix}/{req.name}".replace("//", "/"),
                ObservationWithCovarianceArrayStamped,
                queue_size=1,
                latch=True,
            )
            if is_new_publisher
            else self.publishers[req.name]
        )

        publisher.publish(self.maps[req.name])

        self.publishers[req.name] = publisher

        return SetMapResponse()

    def get_map(self, req: GetMapRequest):
        """
        Gets a map (if it exists)

        Args:
          req: the GetMap request, see GetMap.srv file

        Exceptions:
          ServiceExcepion: when req.name has no map attached to it
        """

        if req.name not in self.maps.keys():
            raise rospy.service.ServiceException(
                f"Namespace '{req.name}' does not have a map attached to it"
            )
        else:
            return GetMapResponse(map=self.maps[req.name])


node = MapServer()
