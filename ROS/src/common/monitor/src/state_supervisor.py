#!/usr/bin/env python3

import signal
import subprocess
import sys

import psutil
import rospkg
import rospy
from node_fixture.fixture import AutonomousStatesEnum, StateMachineScopeEnum
from ugr_msgs.msg import State


class StateSupervisorNode:
    def __init__(self):
        rospy.init_node("state_supervisor", anonymous=True)

        self.mission = rospy.get_param(
            "~mission", "rosbag"
        )  # Default timeout is 10 seconds
        self.record_rosbag = rospy.get_param("~record_rosbag", False)
        if self.record_rosbag:
            rospy.loginfo("Recording rosbag")
            self.rosbag_process = subprocess.Popen(
                "rosbag record -a -j -O {}.bag".format(self.mission),
                stdin=subprocess.PIPE,
                shell=True,
                cwd=rospkg.RosPack().get_path("monitor"),
            )

        # Get parameters for the timeout and topic to monitor
        self.timeout = rospy.get_param(
            "~timeout", 10.0
        )  # Default timeout is 10 seconds

        # Initialize variables
        self.state = AutonomousStatesEnum.ASOFF
        self.start_time = None
        self.rate = rospy.Rate(5)

        # Subscribe to the topic
        rospy.Subscriber("/state", State, self.state_callback)

    def state_callback(self, data: State):
        if data.scope == StateMachineScopeEnum.AUTONOMOUS:
            if data.cur_state == AutonomousStatesEnum.ASDRIVE:
                rospy.loginfo(
                    f"Received '{AutonomousStatesEnum.ASDRIVE}', resetting starting time..."
                )
                self.start_time = data.header.stamp.to_sec()

            self.state = data.cur_state

    def terminate_process_and_children(self, p):
        process = psutil.Process(p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        p.wait()  # we wait for children to terminate

    def stop_recording(self):
        if self.record_rosbag:
            rospy.loginfo("Stopping rosbag recording")
            self.terminate_process_and_children(
                self.rosbag_process
            )  # terminate rosbag process

    def run(self):
        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(
            f"Monitoring state topic for {AutonomousStatesEnum.ASFINISHED} with a timeout of {self.timeout} seconds."
        )

        while not rospy.is_shutdown():
            if self.state == AutonomousStatesEnum.ASFINISHED:
                self.stop_recording()
                rospy.loginfo(
                    f"{AutonomousStatesEnum.ASFINISHED} received in time. Mission took {rospy.Time.now().to_sec() - self.start_time} ROS seconds. Exiting successfully."
                )
                rospy.signal_shutdown("Result received successfully")
                break

            elif self.state == AutonomousStatesEnum.ASEMERGENCY:
                self.stop_recording()
                rospy.logerr(
                    f"{AutonomousStatesEnum.ASEMERGENCY} received. Exiting with an error"
                )
                raise rospy.ROSException(f"Received {AutonomousStatesEnum.ASEMERGENCY}")

            elif (rospy.Time.now().to_sec() - self.start_time) > self.timeout:
                self.stop_recording()
                rospy.logerr(
                    f"Timeout reached. {AutonomousStatesEnum.ASFINISHED} not received. Exiting with an error."
                )
                raise rospy.ROSException("Timeout reached")

            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = StateSupervisorNode()
        node.run()
    except rospy.ROSException:
        sys.exit(1)
