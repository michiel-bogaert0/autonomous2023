import os
import signal
import subprocess

import rospy
from ugr_msgs.msg import TrajectoryMapInfo


class Evaluation:
    """
    Class for handling evaluation.

    Attributes:
        on (bool): Flag indicating whether evaluation is enabled.
        callback (function): Callback function to be called after evaluation.
        running (bool): Flag indicating whether evaluation is currently running.
        process (subprocess.Popen): Subprocess object representing the evaluation process.
    """

    def __init__(self, on: bool, callback):
        """
        Initializes the Evaluation object.

        Args:
            on (bool): Flag indicating whether evaluation is enabled.
            callback (function): Callback function to be called after evaluation.
        """
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
        """
        Launches the evaluation process.
        """
        if self.on:
            self.process = subprocess.Popen(
                "roslaunch slam_trajectory_evaluation trajectory_evaluation.launch printFiles:=False >/dev/null 2>&1",
                stdout=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid,
            )
            self.running = True

    def shutdown(self):
        """
        Terminates the evaluation process.
        """
        if not self.running:
            return
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        self.running = False

    def evaluation_callback(self, data):
        """
        Callback function for the evaluation process.

        Args:
            data: Data received from the evaluation process.
        """
        self.shutdown()
        self.callback(data)
