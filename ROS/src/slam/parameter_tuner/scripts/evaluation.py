import os
import signal
import subprocess

import rospy
from ugr_msgs.msg import TrajectoryMapInfo


class Evaluation:
    def __init__(self, on: bool, callback):
        self.on = on
        self.running = False
        self.callback = callback
        if self.on:
            rospy.Subscriber(
                "/ugr/car/evaluation/infoMap",
                TrajectoryMapInfo,
                self.evaluation_callback,
            )

    def launch(self):
        if self.on:
            self.process = subprocess.Popen(
                "roslaunch slam_trajectory_evaluation trajectory_evaluation.launch printFiles:=False >/dev/null 2>&1",
                stdout=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid,
            )
            self.running = True

    def shutdown(self):
        if not self.running:
            return
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        self.running = False

    def evaluation_callback(self, data):
        rospy.loginfo("Evaluation: %s", data.avgDistanceToConeSLAM)
        self.shutdown()
        self.callback(data)
        # self.got_evaluation = True
        # if self.simulation_finished:
        #     self.handle_end_simulation()
        # else:
        #     rospy.loginfo("Waiting for simulation to finish")
