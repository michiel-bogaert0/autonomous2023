#!/usr/bin/env python3
import can
import rospy
from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from node_fixture import (
    create_diagnostic_message,
    roscan_to_serialcan,
    serialcan_to_roscan,
)

RES_ACTIVATION_MSG = roscan_to_serialcan(
    Frame(id=0x000, data=[0x1, 0x00, 0, 0, 0, 0, 0, 0])
)

ENABLE_AUT_MODE = roscan_to_serialcan(Frame(id=0x400, data=[0, 0, 0, 0, 0, 0, 0, 0]))
ENABLE_RC_MODE = roscan_to_serialcan(Frame(id=0x400, data=[1, 0, 0, 0, 0, 0, 0, 0]))


class CanBridge:
    def __init__(self):
        """
        This node serves as a bridge between a CAN-over-serial interface and ROS

        Args:
          - can_interface: the serial interface to use
        """

        rospy.init_node("can_bridge")

        self.can_interface = rospy.get_param("~can_interface", "can3_ext")
        self.can_bitrate = rospy.get_param("~can_bitrate", 250000)
        self.can_bus = can.Bus(
            interface="socketcan",
            channel=self.can_interface,
            bitrate=self.can_bitrate,
            fd=False,
        )
        self.can_publisher = rospy.Publisher("/output/can", Frame, queue_size=10)
        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        rospy.Subscriber("/input/can", Frame, self.ros_can_callback)

        self.last_time = rospy.Time.now().to_sec()

        self.can_bus.send(RES_ACTIVATION_MSG)

        self.run()

    def ros_can_callback(self, data: Frame):
        """
        Callback to handle CAN messages that need to be written to our CAN bus

        Args:
          data: the Frame that needs to be sent
        """

        can_message = roscan_to_serialcan(data)
        try:
            self.can_bus.send(can_message)
        except Exception:
            rospy.logerr("Is the CAN bus still alive?")

    def run(self):
        """
        This function runs "indefinitely"
        Listens to CAN and publishes all incoming messages to a topic
        """
        received_first_msg = False
        while not rospy.is_shutdown():
            can_message = self.can_bus.recv()
            ros_message = serialcan_to_roscan(can_message)
            ros_message.header.stamp = (
                rospy.Time.now()
            )  # Override timestamp as it is nonsense anyways

            if not received_first_msg:
                self.diagnostics_pub.publish(
                    create_diagnostic_message(
                        DiagnosticStatus.OK,
                        "[MECH] CAN: Driver",
                        "Received first CAN message",
                    )
                )
                received_first_msg = True

            self.can_publisher.publish(ros_message)

            if rospy.Time.now().to_sec() - self.last_time > 0.1:
                try:
                    self.can_bus.send(ENABLE_AUT_MODE)
                    self.last_time = rospy.Time.now().to_sec()
                except Exception:
                    rospy.logerr("Is the CAN bus still alive?")


can_bridge = CanBridge()
