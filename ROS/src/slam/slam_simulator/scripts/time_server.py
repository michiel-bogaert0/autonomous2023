#!/usr/bin/env python3

import time

import rospy
from rosgraph_msgs.msg import Clock
from slam_simulator.srv import (
    SetTimeScaler,
    SetTimeScalerRequest,
    SetTimeScalerResponse,
)


class TimeServer:
    def __init__(self) -> None:
        rospy.init_node("time_simulator", anonymous=True)

        rospy.set_param("/use_sim_time", True)
        self.clock_scaler = rospy.get_param("~time_scaler", 0.1)
        self.clock_freq = rospy.get_param("~rate", 200)

        rospy.Service("set_scaler", SetTimeScaler, self.handle_service)

        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)

        self.time = Clock()
        self.time.clock = rospy.Time.from_sec(time.perf_counter())

    def handle_service(self, req: SetTimeScalerRequest):
        self.clock_scaler = req.scaler
        print("Time scaler set to: ", self.clock_scaler)
        return SetTimeScalerResponse()

    def start(self):
        rospy.loginfo("Time simulator started with time scaler: %f", self.clock_scaler)

        while not rospy.is_shutdown():
            t0 = time.perf_counter()

            self.clock_pub.publish(self.time)

            time.sleep(1 / self.clock_freq)

            t1 = time.perf_counter()
            self.time.clock += rospy.Duration.from_sec((t1 - t0) * self.clock_scaler)


node = TimeServer()
node.start()
