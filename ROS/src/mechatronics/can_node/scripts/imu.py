import rospy
import tf_conversions
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu
from can_msgs.msg import Frame

class ImuConverter:
    def __init__(self):
        # IMU 0 is the FRONT
        self.imu0_pitch_roll = rospy.Publisher(
            "/output/imu0/pitch_roll",
            Imu,
            queue_size=10,
        )
        self.imu0_angular_rate = rospy.Publisher(
            "/output/imu0/angular_rate",
            Imu,
            queue_size=10,
        )
        self.imu0_acc = rospy.Publisher(
            "/output/imu0/acc",
            Imu,
            queue_size=10,
        )

        # IMU 1 is the BACK
        self.imu1_pitch_roll = rospy.Publisher(
            "/output/imu1/pitch_roll",
            Imu,
            queue_size=10,
        )
        self.imu1_angular_rate = rospy.Publisher(
            "/output/imu1/angular_rate",
            Imu,
            queue_size=10,
        )
        self.imu1_acc = rospy.Publisher(
            "/output/imu1/acc",
            Imu,
            queue_size=10,
        )

        self.imu0_frame = rospy.get_param(
            "~imu0/frame", "ugr/car_base_link/imu0"
        )
        self.imu1_frame = rospy.get_param(
            "~imu1/frame", "ugr/car_base_link/imu1"
        )

    def handle_imu_msg(self, msg: Frame, is_front: bool) -> None:
        """Publishes an IMU message to the correct ROS topic

        Args:
            msg: the CAN message
            is_front: whether the message came from the front IMU or not
        """
        imu_msg = Imu()

        latency = msg.data[7]
        ts_corrected = rospy.Time.from_sec(
            msg.header.stamp.to_sec() - 0.0025 if latency == 5 else msg.header.stamp.to_sec()
        )  # 2.5 ms latency according to the datasheet
        imu_msg.header.stamp = ts_corrected
        imu_msg.header.frame_id = (
            self.imu0_frame if is_front else self.imu1_frame
        )

        # Decode the ID to figure out the type of message
        # We need the middle 4*8 bits of the ID to determine the type of message
        # IMU reports deg, while ROS expects rad
        cmd_id = (msg.id & 0x00FFFF00) >> 8
        if cmd_id == 0xF029:
            # Pitch and roll
            pitch_raw = (msg.data[2] << 16) + (msg.data[1] << 8) + msg.data[0]
            pitch = (pitch_raw - 8192000) / 32768 * np.pi / 180

            roll_raw = (msg.data[5] << 16) + (msg.data[4] << 8) + msg.data[3]
            roll = (roll_raw - 8192000) / 32768 * np.pi / 180

            imu_msg.orientation = Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(roll, pitch, 0)
            )
            imu_msg.orientation_covariance = (
                np.diag([1, 1, 1]).reshape((1, 9)).tolist()[0]
            )  # TODO change

            if is_front:
                self.imu0_pitch_roll.publish(imu_msg)
                return
            self.imu1_pitch_roll.publish(imu_msg)

        elif cmd_id == 0xF02A:
            # Angular rate
            # IMU reports in deg/s, while ROS expects rad/s (see REP103)
            pitch_raw = (msg.data[1] << 8) + msg.data[0]
            pitch = (pitch_raw - 32000) / 128 # deg/s

            roll_raw = (msg.data[3] << 8) + msg.data[2]
            roll = (roll_raw - 32000) / 128

            yaw_raw = (msg.data[5] << 8) + msg.data[4]
            yaw = (yaw_raw - 32000) / 128

            rate = Vector3()
            rate.x = roll * np.pi / 180 # rad/s
            rate.y = pitch * np.pi / 180
            rate.z = yaw * np.pi / 180
            imu_msg.angular_velocity = rate
            imu_msg.angular_velocity_covariance = (
                np.diag([1, 1, 1]).reshape((1, 9)).tolist()[0]
            )  # TODO change

            if is_front:
                self.imu0_angular_rate.publish(imu_msg)
                return
            self.imu1_angular_rate.publish(imu_msg)

        elif cmd_id == 0xF02D:
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
            imu_msg.linear_acceleration = acc
            imu_msg.linear_acceleration_covariance = (
                np.diag([1, 1, 1]).reshape((1, 9)).tolist()[0]
            )  # TODO change

            if is_front:
                self.imu0_acc.publish(imu_msg)
                return
            self.imu1_acc.publish(imu_msg)
