import can
import rospy
import tf_conversions
from geometry_msgs.msg import Vector3
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
            "~imu/front/frame", ""
        )  # TODO welke hier?
        self.imu_back_frame = rospy.get_param("~imu/back/frame", "")  # TODO welke hier?

    def handle_imu_msg(self, msg: can.Message, is_front: bool) -> None:
        """Publishes an IMU message to the correct ROS topic

        Args:
            msg: the CAN message
            is_front: whether the message came from the front IMU or not
        """
        status = msg.buf[6]
        # Check for errors
        if status != 0xFF:
            rospy.logerr(f"Message (id {msg.id}) contained errors, status: {status}")
            return

        imu_msg = Imu()

        latency = msg.buf[7]
        ts_corrected = (
            msg.timestamp - 0.0025 if latency == 5 else msg.timestamp
        )  # 2.5 ms latency according to the datasheet
        imu_msg.header.stamp = ts_corrected
        imu_msg.header.frame_id = (
            self.imu_front_frame if is_front else self.imu_back_frame
        )

        # Decode the ID to figure out the type of message
        # We need the first 24 bits of the ID
        cmd_id = (msg.id & 0x00FFFFFF) >> 2
        if cmd_id == 0xF029:
            # Pitch and roll
            pitch_raw = (msg.buf[2] << 16) + (msg.buf[1] << 8) + msg.buf[0]
            pitch = (pitch_raw - 8192000) / 32768

            roll_raw = (msg.buf[5] << 16) + (msg.buf[4] << 8) + msg.buf[3]
            roll = (roll_raw - 8192000) / 32768

            imu_msg.orientation = tf_conversions.transformations.quaternion_from_euler(
                roll, pitch, 0
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
            pitch_raw = (msg.buf[1] << 8) + msg.buf[0]
            pitch = (pitch_raw - 32000) / 128

            roll_raw = (msg.buf[3] << 8) + msg.buf[2]
            roll = (roll_raw - 32000) / 128

            yaw_raw = (msg.buf[5] << 8) + msg.buf[4]
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

        elif 0xF02DE2:
            # Acceleration
            lat_raw = (msg.buf[1] << 8) + msg.buf[0]
            lat = (lat_raw - 32000) / 100

            lon_raw = (msg.buf[3] << 8) + msg.buf[2]
            lon = (lon_raw - 32000) / 100

            vert_raw = (msg.buf[5] << 8) + msg.buf[4]
            vert = (ert_raw - 32000) / 100

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
