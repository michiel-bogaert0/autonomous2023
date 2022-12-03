#!/usr/bin/env python3
import struct

import can
import cantools
import numpy as np
import rospy
from std_msgs.msg import Empty
from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from imu import ImuConverter
from odrive import OdriveConverter
from node_fixture.node_fixture import create_diagnostic_message

RES_ACTIVATION_MSG = can.Message(
    arbitration_id=0x000,
    data=[0x1, 0x11, 0, 0, 0, 0, 0, 0],
    is_extended_id=False,
)


class CanConverter:
    def __init__(self):
        rospy.init_node("can_converter")

        self.can_pub = rospy.Publisher("/output/can", Frame, queue_size=10)
        self.diagnostics = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        self.start_pub = rospy.Publisher("/output/start", Empty, queue_size=10)
        self.stop_pub = rospy.Publisher("/output/stop", Empty, queue_size=10)
        self.reset_pub = rospy.Publisher("/output/reset", Empty, queue_size=10)

        self.res_send_interval = rospy.get_param("~res_send_interval", 0.1)  # ms

        # The first element is the front IMU, the second is the rear IMU
        self.IMU_IDS = [0xE2, 0xE3]

        # create a bus instance
        self.bus = can.interface.Bus(
            bustype="socketcan",
            channel=rospy.get_param("~interface", "can0"),
            bitrate=rospy.get_param("~baudrate", 250000),
        )

        # Create the right converters
        self.odrive = OdriveConverter()
        self.imu = ImuConverter()

        # Activate the RES signals
        self.res_activated = False
        self.last_send_time = rospy.get_time()
        self.bus.send(RES_ACTIVATION_MSG)

        try:
            self.listen_on_can()
        except rospy.ROSInterruptException:
            pass

    def listen_on_can(self) -> None:
        """Listens on the can interface of the Jetson and publishes the messages"""

        # iterate over received messages
        # this keeps looping forever
        for msg in self.bus:
            # Publish message on ROS
            can_msg = Frame()
            can_msg.header.stamp = rospy.Time.from_sec(msg.timestamp)
            can_msg.id = msg.arbitration_id
            can_msg.data = msg.data
            self.can_pub.publish(can_msg)

            # Check if the message is a ODrive command
            axis_id = msg.arbitration_id >> 5
            if axis_id == 1 or axis_id == 2:
                cmd_id = msg.arbitration_id & 0b11111

                if cmd_id == 9:
                    self.diagnostics.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name="[Mechatronics] CAN converter (ODrive)",
                            message="ODrive message received.",
                        )
                    )
                    self.odrive.handle_vel_msg(msg, axis_id, cmd_id)
                    continue

            imu_id = msg.arbitration_id & 0xFF
            if imu_id in self.IMU_IDS:
                status = msg.data[6]
                # Check for errors
                if status != 0x00:
                    rospy.logerr(
                        f"Message (id {msg.arbitration_id}) contained errors, status: {status}"
                    )

                    self.diagnostics.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.WARN,
                            name=f"[Mechatronics] CAN converter (IMU [{imu_id}])",
                            message="IMU generated faulty message.",
                        )
                    )
                    continue

                self.diagnostics.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name=f"[Mechatronics] CAN converter (IMU [{imu_id}])",
                            message="IMU operational.",
                        )
                    )
                self.imu.handle_imu_msg(msg, imu_id == self.IMU_IDS[0])

            if msg.arbitration_id == 0x191:
                self.diagnostics.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name="[Mechatronics] CAN converter (RES)",
                            message="RES activated.",
                        )
                    )
                self.res_activated = True

                switch = (msg.data[0] & 0b0000010) >> 1
                button = (msg.data[0] & 0b0000100) >> 2
                if switch:
                    self.start_pub.publish(Empty())
                    self.diagnostics.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name="[Mechatronics] RES",
                            message="START",
                        )
                    )
                else:
                    self.stop_pub.publish(Empty())
                    self.diagnostics.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.WARN,
                            name="[Mechatronics] RES",
                            message="STOP",
                        )
                    )

                if button:
                    self.reset_pub.publish(Empty())

            elif (
                not self.res_activated
                and rospy.get_time() - self.last_send_time > self.res_send_interval
            ):
                self.last_send_time = rospy.get_time()
                self.bus.send(RES_ACTIVATION_MSG)

            # Check for external shutdown
            if rospy.is_shutdown():
                return

    def send_on_can(self, msg: can.Message) -> None:
        """Sends a message msg over the can bus

        Args:
            msg: the message to send

        """
        try:
            self.bus.send(msg)
            rospy.loginfo(f"Message sent on {self.bus.channel_info}")
        except can.CanError:
            rospy.logerr("Message NOT sent")


if __name__ == "__main__":
    try:
        cp = CanConverter()
    except rospy.ROSInterruptException:
        pass
