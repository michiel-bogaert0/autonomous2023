#!/usr/bin/env python3
import rospy
from can_msgs.msg import Frame
from can.interfaces.serial.serial_can import SerialBus
from can import Message
from node_fixture import serialcan_to_roscan, roscan_to_serialcan


class CanBridge:
    def __init__(self):
        """
        This node serves as a bridge between a CAN-over-serial interface and ROS

        Args:
          - can_interface: the serial interface to use
        """

        rospy.init_node("can_bridge")

        self.can_interface = rospy.get_param("~can_interface", "/dev/ttyACM0")
        self.can_bus = SerialBus(self.can_interface)

        self.can_publisher = rospy.Publisher("/output/can", Frame, queue_size=10)
        rospy.Subscriber("/input/can", Frame, self.ros_can_callback)

        self.run()

    def ros_can_callback(self, data: Frame):
        """
        Callback to handle CAN messages that need to be written to our CAN bus

        Args:
          data: the Frame that needs to be sent
        """

        can_message = roscan_to_serialcan(data)
        self.can_bus.send(can_message)

    def run(self):
        """
        This function runs "indefinitely"
        Listens to CAN and publishes all incoming messages to a topic
        """

        while not rospy.is_shutdown():
            can_message = self.can_bus.recv()
            ros_message = serialcan_to_roscan(can_message)

            self.can_publisher.publish(ros_message)


can_bridge = CanBridge()
