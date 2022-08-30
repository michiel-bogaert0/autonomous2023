#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import AddSubscriber, ROSNode
from pid import PID
from trajectory import Trajectory
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose
import tf2_ros as tf
from tf.transformations import euler_from_quaternion
from fs_msgs.msg import ControlCommand


class PIDControlNode(ROSNode):
    def __init__(self):
        super().__init__("pid_car_control")

        # PID settings
        Kp = rospy.get_param("~steering/Kp", 1.0)
        Ki = rospy.get_param("~steering/Ki", 0.0)
        Kd = rospy.get_param("~steering/Kd", 0.0)

        self.max_steering_angle = rospy.get_param(
            "~steering/max_steering_angle", 45
        )  # in deg

        self.steering_pid = PID(Kp, Ki, Kd, reset_rotation=True)

        self.publish_rate = rospy.get_param("~publish_rate", 10.0)

        self.speed_target = rospy.get_param("~speed/target", 3)

        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/car_odom")

        self.cmd = ControlCommand(steering=0.0, throttle=0.0, brake=1.0)

        self.current_angle = 0
        self.current_pos = [0, 0]
        self.set_angle = 0

        self.current_path = np.zeros((0, 2))

        """
          Trajectory parameters and conditions
            - minimal_distance: the minimal required distance between the car and the candidate target point
            - max_angle: the maximal allowed angle difference between the car and the candidate target point
            - t_step: the t step the alg takes when progressing through the underlying parametric equations 
                      Indirectly determines how many points are checked per segment. 
        """
        minimal_distance = rospy.get_param("trajectory/minimal_distance", 4)
        t_step = rospy.get_param("trajectory/t_step", 0.05)
        max_angle = rospy.get_param("trajectory/max_angle", 1)
        self.trajectory = Trajectory(minimal_distance, t_step, max_angle)

        # Helpers
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        self.start_pid_sender()

    @AddSubscriber("/input/path")
    def getPathplanningUpdate(self, msg: PoseArray):
        """
        Takes in a new exploration path coming from the mapping algorithm.
        The path should be relative to self.world_frame. Otherwise it will transform to it
        """

        pose_array_transformed = msg

        if msg.header.frame_id != self.world_frame:

            # Transform received message
            trans = self.tf_buffer.lookup_transform_full(
                self.world_frame,
                rospy.Time(),
                msg.header.frame_id,
                msg.header.stamp,
                self.world_frame,
            )
            new_header = Header(frame_id=self.world_frame, stamp=rospy.Time.now())
            pose_array_transformed = PoseArray(header=new_header)
            for pose in msg.poses:
                pose_s = PoseStamped(pose=pose, header=msg.header)
                pose_t = do_transform_pose(pose_s, trans)
                pose_array_transformed.poses.append(pose_t.pose)

        # Create a new path
        self.current_path = np.zeros((0, 2))
        for pose in pose_array_transformed.poses:
            self.current_path = np.vstack(
                (self.current_path, [pose.position.x, pose.position.y])
            )

        self.trajectory.set_path(self.current_path)

    def start_pid_sender(self):
        """
        Start sending updates. If the data is too old, brake.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():

            # Look up the base_link frame relative to the world at this point in time.
            # This is needed to set the current position of the car
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_link_frame, 0
            )

            _, _, yaw = euler_from_quaternion(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            )

            self.current_pos = [
                transform.transform.translation.x,
                transform.transform.translation.y,
            ]
            self.current_angle = yaw

            # First try to get a target point
            target_x, target_y, success = self.trajectory.calculate_target_point(
                self.current_pos, self.current_angle
            )

            if not success:
                # BRAKE! We don't know where to drive to!
                rospy.loginfo("No target point found!")
                self.cmd.brake = 1.0
                self.cmd.throttle = 0.0
            else:
                # Go ahead and drive
                self.cmd.brake = 0.0
                self.cmd.throttle = 1.0

                msg = PoseStamped()
                msg.header.frame_id = self.base_link_frame
                msg.header.stamp = rospy.Time.now()

                msg.pose.position = Point(target_x, target_y, 0)

                self.publish("/output/target_point", msg)

                # Calculate angle
                self.set_angle = PID.pi_to_pi(
                    np.arctan2(
                        target_y - self.current_pos[1], target_x - self.current_pos[0]
                    )
                )

                # PID step
                error_pid = self.steering_pid(self.current_angle - self.set_angle)
                rospy.loginfo(
                    f"target: {target_x} - {target_y} from {self.current_pos}"
                )
                rospy.loginfo(
                    f"Steering: {error_pid:.3f} from {PID.pi_to_pi(self.current_angle - self.set_angle):.3f} - c:{self.current_angle:.3f} - s:{self.set_angle:.3f}"
                )

                # Remap PID to [-1, 1]
                error_pid = min(
                    np.deg2rad(self.max_steering_angle),
                    max(-np.deg2rad(self.max_steering_angle), error_pid),
                )
                old_range = np.deg2rad(self.max_steering_angle) * 2
                new_range = 2
                error_pid = (
                    (error_pid + np.deg2rad(self.max_steering_angle))
                    * new_range
                    / old_range
                ) - 1

                self.cmd.steering = error_pid

            self.publish("/output/control_command", self.cmd)
            rate.sleep()


node = PIDControlNode()
node.start()
