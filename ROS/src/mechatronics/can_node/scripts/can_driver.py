#!/usr/bin/env python3
import rospy
from can_msgs.msg import Frame
from can.interfaces.serial.serial_can import SerialBus
from can import Message


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

        can_message = Message(
            timestamp=data.header.stamp.to_sec(),
            extended_id=data.is_extended,
            is_error_frame=data.is_error,
            dlc=data.dlc,
            arbitration_id=data.id,
            data=list(data.data),
        )
        self.can_bus.send(can_message)

    def run(self):
        """
        This function runs "indefinitely"
        Listens to CAN and publishes all incoming messages to a topic
        """

        while not rospy.is_shutdown():
            can_message = self.can_bus.recv()
            ros_message = Frame()
            ros_message.header.stamp = rospy.Time.from_sec(can_message.timestamp)
            ros_message.id = can_message.arbitration_id
            ros_message.data = bytearray(can_message.data)
            ros_message.dlc = can_message.dlc
            ros_message.is_error = can_message.is_error_frame
            ros_message.is_extended = can_message.is_extended_id

            self.can_publisher.publish(ros_message)


can_bridge = CanBridge()
