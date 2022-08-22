#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray, Point, PoseStamped
from nav_msgs.msg import Odometry
from node_fixture import AddSubscriber, ROSNode
from pid import PID
from tf.transformations import euler_from_quaternion
from trajectory import Trajectory
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose
import tf2_ros as tf
from fs_msgs.msg import ControlCommand


class PIDControlNode(ROSNode):
    def __init__(self):
        super().__init__("pid_car_control")

        # PID settings
        Kp = rospy.get_param("car_control/steering/Kp", 0.01)
        Ki = rospy.get_param("car_control/steering/Ki", 0.0)
        Kd = rospy.get_param("car_control/steering/Kd", 0.0)

        self.steering_pid = PID(Kp, Ki, Kd, reset_rotation=True)

        self.publish_rate = rospy.get_param("~car_control/publish_rate", 10.0)
        self.missed_updates_till_bad = int(
            self.publish_rate * rospy.get_param("~car_control/stale_time", 0.2)
        )
        self.speed_target = rospy.get_param("~car_control/speed/target", 3)

        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/car_odom")

        self.stale = True
        self.cmd = ControlCommand(steering=0.0, throttle=0.0, brake=1.0)
        self.missed_updates = 0

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
        minimal_distance = rospy.get_param("car_control/trajectory/minimal_distance", 4)
        t_step = rospy.get_param("car_control/trajectory/t_step", 0.05)
        max_angle = rospy.get_param("car_control/trajectory/max_angle", 1)
        self.trajectory = Trajectory(minimal_distance, t_step, max_angle)

        # Helpers
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

    @AddSubscriber("/input/path")
    def getPathplanningUpdate(self, msg: PoseArray):
        """
        Takes in a new exploration path coming from the mapping algorithm
        """
        # Received an update so the ControlCommand isn't stale
        if self.stale:
            self.stale = False
            self.missed_updates = 0

        # Transform received message
        trans = self.tf_buffer.lookup_transform_full(
            self.base_link_frame,
            rospy.Time(),
            msg.header.frame_id,
            msg.header.stamp,
            self.world_frame,
        )
        new_header = Header(frame_id=self.base_link_frame, stamp=rospy.Time.now())
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

    def start_pid_sender(self, event):
        """
        Start sending updates. If the data is too old, brake.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():

            # Shortcut for when system is stale
            if self.stale:
                self.publish(
                    "/output/drive_command",
                    ControlCommand(steering=0.0, throttle=0.0, brake=1.0),
                )
                rate.sleep()
                continue
            else:
                self.missed_updates += 1

                if self.missed_updates > self.missed_updates_till_bad:
                    # Stale data: BRAKE
                    self.publish(
                        "/output/drive_command",
                        ControlCommand(steering=0.0, throttle=0.0, brake=1.0),
                    )
                    self.stale = True
                    print("System became stale")
                    continue

            # First try to set the set angle
            target_x, target_y, success = self.trajectory.calculate_target_point(
                self.current_pos, self.current_angle
            )

            if not success:
                print("No target point found!")

            self.publish("/output/target_point", Point(target_x, target_y, 0))

            # Calculate angle
            self.set_angle = PID.pi_to_pi(
                np.arctan2(
                    target_y - self.current_pos[1], target_x - self.current_pos[0]
                )
            )

            # PID step
            error_pid = self.steering_pid(self.current_angle - self.set_angle)
            print(f"target: {target_x} - {target_y} from {self.current_pos}")
            print(
                f"Steering: {error_pid:.3f} from {PID.pi_to_pi(self.current_angle - self.set_angle):.3f} - c:{self.current_angle:.3f} - s:{self.set_angle:.3f}"
            )

            # Remap PID to [-1, 1]
            error_pid = min(np.deg2rad(45), max(-np.deg2rad(45), error_pid))
            old_range = np.deg2rad(45) * 2
            new_range = 2
            error_pid = ((error_pid + np.deg2rad(45)) * new_range / old_range) - 1

            self.cmd.steering = error_pid

            self.publish("/output/drive_command", self.cmd)
            rate.sleep()


node = PIDControlNode()
timer = rospy.Timer(
    rospy.Duration(1 / rospy.get_param("car_control/publish_rate", 100)),
    node.start_pid_sender,
)
node.start()
timer.shutdown()  # Normally never gets called, because node.start() blocks!
