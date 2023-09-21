#!/usr/bin/env python3

import rospy
from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from imu import ImuConverter
from node_fixture.node_fixture import create_diagnostic_message
from odrive import OdriveConverter


class CanConverter:
    def __init__(self):
        # Activate the RES signals
        self.res_activated = False

        rospy.init_node("can_converter")

        self.last_send_time = rospy.get_time()

        self.diagnostics = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        self.res_send_interval = rospy.get_param("~res_send_interval", 0.1)  # ms

        # The first element is the front IMU, the second is the rear IMU
        self.IMU_IDS = [0xE2, 0xE3]

        self.bus = rospy.Publisher("/output/can", Frame, queue_size=10)

        # Create the right converters
        self.odrive = OdriveConverter(self.bus)
        self.imu = ImuConverter()

        # create a can subscriber
        rospy.Subscriber("/input/can", Frame, self.listen_on_can)

        rospy.spin()

    def listen_on_can(self, can_msg) -> None:
        """
        Subscriber callback for can messages

        Args:
            - can_msg: the Frame message coming from our CAN bus
        """

        # Check if the message is a ODrive command
        axis_id = can_msg.id >> 5
        if axis_id == 1 or axis_id == 2:
            cmd_id = can_msg.id & 0b11111

            if cmd_id == 9:
                self.diagnostics.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.OK,
                        name="[MECH] CAN: ODrive",
                        message="ODrive message received.",
                    )
                )
                self.odrive.handle_vel_msg(can_msg, axis_id, cmd_id)

            elif cmd_id == 23:
                self.diagnostics.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.OK,
                        name="[MECH] CAN: ODrive",
                        message="ODrive power message received.",
                    )
                )
                self.odrive.handle_power_msg(can_msg, cmd_id)
        imu_id = can_msg.id & 0xFF
        if imu_id in self.IMU_IDS:
            status = can_msg.data[6]
            # Check for errors
            if status != 0x00:
                rospy.logerr(
                    f"Message (id {can_msg.id}) contained errors, status: {status}"
                )

                self.diagnostics.publish(
                    create_diagnostic_message(
                        level=DiagnosticStatus.WARN,
                        name=f"[MECH] CAN: IMU [{imu_id}]",
                        message="IMU generated faulty message.",
                    )
                )

            self.diagnostics.publish(
                create_diagnostic_message(
                    level=DiagnosticStatus.OK,
                    name=f"[MECH] CAN: IMU [{imu_id}]",
                    message="IMU operational.",
                )
            )
            self.imu.handle_imu_msg(can_msg, imu_id == self.IMU_IDS[0])

        if can_msg.id == 0x191:
            self.diagnostics.publish(
                create_diagnostic_message(
                    level=DiagnosticStatus.OK,
                    name="[MECH] CAN: RES",
                    message="RES activated.",
                )
            )
            self.res_activated = True

        elif (
            not self.res_activated
            and rospy.get_time() - self.last_send_time > self.res_send_interval
        ):
            self.last_send_time = rospy.get_time()


if __name__ == "__main__":
    try:
        cp = CanConverter()
    except rospy.ROSInterruptException:
        pass
