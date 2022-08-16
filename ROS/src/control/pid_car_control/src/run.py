#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
from node_fixture import AddSubscriber, ROSNode
from pid import PID
from tf.transformations import euler_from_quaternion
from trajectory import Trajectory
from ugr_msgs.msg import DriveCommand, ExplorationPath

pub = rospy.Publisher("/processed/drive_command", DriveCommand, queue_size=10)


class PIDControlNode(ROSNode):
    def __init__(self):
        super().__init__("pid_car_control")

        # PID settings
        Kp = rospy.get_param("car_control/steering/Kp", 2.0)
        Ki = rospy.get_param("car_control/steering/Ki", 0.1)
        Kd = rospy.get_param("car_control/steering/Kd", 0.2)

        self.steering_pid = PID(Kp, Ki, Kd, reset_rotation=True)

        Kp = rospy.get_param("car_control/speed/Kp", 2.0)
        Ki = rospy.get_param("car_control/speed/Ki", 0.0)
        Kd = rospy.get_param("car_control/speed/Kd", 0.0)

        self.speed_pid = PID(Kp, Ki, Kd)

        self.publish_rate = rospy.get_param("car_control/publish_rate", 1.0)
        self.missed_updates_till_bad = int(self.publish_rate * rospy.get_param("car_control/stale_time", 0.2))
        self.speed_target = rospy.get_param("car_control/speed/target", 3)

        self.stale = True
        self.cmd = DriveCommand(steering=0.0, throttle=0.0, brake=1.0)
        self.missed_updates = 0

        self.current_speed = 0
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

    # TODO: should be set to a good source of velocity, sensor fusion doesn't seem accurate
    @AddSubscriber("/fsds/gss")
    def getCarLocalPosition(self, msg: TwistStamped):
        # Update car speed
        velocity = msg.twist.linear
        self.current_speed = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** (1 / 2)

    @AddSubscriber("/odometry/filtered/global")
    def getCarGlobalPosition(self, odomMsg: Odometry):
        """
        Takes in state information of the car (GPS + IMU)
        """
        # Get car position and rotation to be able to transform the cones
        orientation_q = odomMsg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.current_angle = yaw
        self.current_pos = [odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y]

    @AddSubscriber("/exploration_path")
    def getExplorationPath(self, msg: ExplorationPath):
        """
        Takes in a new exploration path coming from the mapping algorithm
        """
        # Received an update so the DriveCommand isn't stale
        if self.stale:
            self.stale = False
            self.missed_updates = 0

        self.current_path = np.zeros((0, 2))
        for point in msg.exploration_path:
            self.current_path = np.vstack((self.current_path, [point.x, point.y]))

        # Also apply a transformation
        yaw = self.current_angle
        R = np.array([[np.cos(yaw), -1 * np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        tf_points = R.dot(np.array(self.current_path).T).T
        tf_points[:, 0] += self.current_pos[0]
        tf_points[:, 1] += self.current_pos[1]

        self.current_path = tf_points

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
                    "/processed/drive_command",
                    DriveCommand(steering=0.0, throttle=0.0, brake=1.0),
                )
                rate.sleep()
                continue
            else:
                self.missed_updates += 1

                if self.missed_updates > self.missed_updates_till_bad:
                    # Stale data: BRAKE
                    self.publish(
                        "/processed/drive_command",
                        DriveCommand(steering=0.0, throttle=0.0, brake=1.0),
                    )
                    self.stale = True
                    print("System became stale")
                    continue

            # First try to set the set angle
            target_x, target_y, success = self.trajectory.calculate_target_point(self.current_pos, self.current_angle)

            if not success:
                print("No target point found!")

            self.publish("/pid_car_control/target_point", Point(target_x, target_y, 0))

            # Calculate angle
            self.set_angle = PID.pi_to_pi(np.arctan2(target_y - self.current_pos[1], target_x - self.current_pos[0]))

            # PID step
            error_pid = self.steering_pid(self.current_angle - self.set_angle)
            print(f"target: {target_x} - {target_y} from {self.current_pos}")
            print(
                f"Steering: {error_pid:.3f} from {PID.pi_to_pi(self.current_angle - self.set_angle):.3f} - c:{self.current_angle:.3f} - s:{self.set_angle:.3f}"
            )

            speed_pid = self.speed_pid(self.speed_target - self.current_speed)
            print(f"Speed:    {speed_pid:.3f} from {(self.speed_target - self.current_speed):.3f}")

            # Remap PID to [-1, 1]
            error_pid = min(np.deg2rad(45), max(-np.deg2rad(45), error_pid))
            old_range = np.deg2rad(45) * 2
            new_range = 2
            error_pid = ((error_pid + np.deg2rad(45)) * new_range / old_range) - 1

            # TODO: This model doesn't recognise that you can still slow down by friction (aka without braking)
            # Quick fix is to map slight braking values to no braking
            speed_pid = 0 if -0.3 < speed_pid < 0 else speed_pid

            throttle = 0
            brake = 0
            if speed_pid < 0:
                brake = speed_pid
            else:
                throttle = speed_pid

            self.cmd.throttle = throttle
            self.cmd.brake = brake
            self.cmd.steering = error_pid

            self.publish("/processed/drive_command", self.cmd)
            rate.sleep()


node = PIDControlNode()
timer = rospy.Timer(
    rospy.Duration(1 / rospy.get_param("car_control/publish_rate", 0.01)),
    node.start_pid_sender,
)
node.start()
timer.shutdown()  # Normally never gets called, because node.start() blocks!