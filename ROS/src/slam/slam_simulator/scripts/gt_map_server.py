#!/usr/bin/env python3

import rospy
from slam_simulator.srv import (
    GetGTMap,
    GetGTMapRequest,
    GetGTMapResponse    
)
from fs_msgs.msg import Cone
import yaml


class GTMapServer:
    """
    GTMapServer class for handling the ugr/srv/get_gt_map service used by the Unity Simulator.
    """
    def __init__(self):
        rospy.init_node('gt_map_')
        self.map = rospy.get_param('~map', 'chicane')
        self.service = rospy.Service('ugr/srv/get_gt_map', GetGTMap, self.handle_service)

    def handle_service(self, req):
        print(f'Getting GT map for {self.map}')

        try:
            with open(f'/home/ugr/autonomous2023/ROS/src/slam/slam_simulator/maps/{self.map}.yaml', 'r') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            rospy.logerr(f'File not found: {self.map}')
            return GetGTMapResponse([])

        cones = []

        for observation in data['observations']:
            cone = Cone()
            cone.location.x = observation['observation']['location']['x']
            cone.location.y = observation['observation']['location']['y']
            cone.location.z = observation['observation']['location']['z']
            cone.color = observation['observation']['observation_class']
            cones.append(cone)

        return GetGTMapResponse(cones) 


server = GTMapServer()
rospy.spin()