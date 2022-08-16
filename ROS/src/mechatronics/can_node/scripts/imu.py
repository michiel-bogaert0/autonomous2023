import can
import rospy
import tf_conversions
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu


class ImuConverter:
    def __init__(self):
        self.imu_front_pitch_roll = rospy.Publisher(
            "/output/imu0/pitch_roll",
            Imu,
            queue_size=10,
        )
        self.imu_front_angular_rate = rospy.Publisher(
            "/output/imu0/angular_rate",
            Imu,
            queue_size=10,
        )
        self.imu_front_acc = rospy.Publisher(
            "/output/imu0/acc",
            Imu,
            queue_size=10,
        )
        self.imu_back_pitch_roll = rospy.Publisher(
            "/output/imu1/pitch_roll",
            Imu,
            queue_size=10,
        )
        self.imu_back_angular_rate = rospy.Publisher(
            "/output/imu1/angular_rate",
            Imu,
            queue_size=10,
        )
        self.imu_back_acc = rospy.Publisher(
            "/output/imu1/acc",
            Imu,
            queue_size=10,
        )

        self.imu_front_frame = rospy.get_param(
            "~imu/front/frame", "ugr/car_base_link/imu1"
        )
        self.imu_back_frame = rospy.get_param(
            "~imu/back/frame", "ugr/car_base_link/imu0"
        )

    def handle_imu_msg(self, msg: can.Message, is_front: bool) -> None:
        """Publishes an IMU message to the correct ROS topic

        Args:
            msg: the CAN message
            is_front: whether the message came from the front IMU or not
        """
        status = msg.data[6]
        # Check for errors
        if status != 0x00:
            rospy.logerr(
                f"Message (id {msg.arbitration_id}) contained errors, status: {status}"
            )
            return

        imu_msg = Imu()

        latency = msg.data[7]
        ts_corrected = rospy.Time.from_sec(
            msg.timestamp - 0.0025 if latency == 5 else msg.timestamp
        )  # 2.5 ms latency according to the datasheet
        imu_msg.header.stamp = ts_corrected
        imu_msg.header.frame_id = (
            self.imu_front_frame if is_front else self.imu_back_frame
        )

        # Decode the ID to figure out the type of message
        # We need the middle 4*8 bits of the ID to determine the type of message
        cmd_id = (msg.arbitration_id & 0x00FFFF00) >> 8
        if cmd_id == 0xF029:
            # Pitch and roll
            pitch_raw = (msg.data[2] << 16) + (msg.data[1] << 8) + msg.data[0]
            pitch = (pitch_raw - 8192000) / 32768

            roll_raw = (msg.data[5] << 16) + (msg.data[4] << 8) + msg.data[3]
            roll = (roll_raw - 8192000) / 32768

            imu_msg.orientation = Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(roll, pitch, 0)
            )
            imu_msg.orientation_covariance = (
                np.diag([1, 1, 1]).reshape((1, 9)).tolist()[0]
            )  # TODO change

            if is_front:
                self.imu_front_pitch_roll.publish(imu_msg)
                return
            self.imu_back_pitch_roll.publish(imu_msg)

        elif 0xF02A:
            # Angular rate
            pitch_raw = (msg.data[1] << 8) + msg.data[0]
            pitch = (pitch_raw - 32000) / 128

            roll_raw = (msg.data[3] << 8) + msg.data[2]
            roll = (roll_raw - 32000) / 128

            yaw_raw = (msg.data[5] << 8) + msg.data[4]
            yaw = (yaw_raw - 32000) / 128

            rate = Vector3()
            rate.x = roll
            rate.y = pitch
            rate.z = yaw
            imu_msg.angular_velocity = rate
            imu_msg.angular_velocity_covariance = (
                np.diag([1, 1, 1]).reshape((1, 9)).tolist()[0]
            )  # TODO change

            if is_front:
                self.imu_front_angular_rate.publish(imu_msg)
                return
            self.imu_back_angular_rate.publish(imu_msg)

        if cmd_id == 0xF02D:
            # Acceleration
            lat_raw = (msg.data[1] << 8) + msg.data[0]
            lat = (lat_raw - 32000) / 100

            lon_raw = (msg.data[3] << 8) + msg.data[2]
            lon = (lon_raw - 32000) / 100

            vert_raw = (msg.data[5] << 8) + msg.data[4]
            vert = (vert_raw - 32000) / 100

            acc = Vector3()
            acc.x = lon
            acc.y = lat
            acc.z = vert
            imu_msg.angular_velocity = acc
            imu_msg.angular_velocity_covariance = (
                np.diag([1, 1, 1]).reshape((1, 9)).tolist()[0]
            )  # TODO change

            if is_front:
                self.imu_front_acc.publish(imu_msg)
                return
            self.imu_back_acc.publish(imu_msg)
