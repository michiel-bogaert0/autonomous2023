#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from node_fixture.fixture import (
    DiagnosticArray,
    DiagnosticStatus,
    create_diagnostic_message,
)
from std_msgs.msg import Float64, Header
from tf2_geometry_msgs import do_transform_pose
from trajectory import Trajectory


class PurePursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit_control")

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.min_speed = rospy.get_param("~speed/min", 0.3)
        self.min_corner_speed = rospy.get_param("~speed/min_corner_speed", 0.7)
        self.wheelradius = rospy.get_param("~wheelradius", 0.1)

        self.velocity_cmd = Float64(0.0)
        self.steering_cmd = Float64(0.0)
        self.actual_speed = 0.0
        self.speed_start = rospy.get_param("~speed_start", 10)
        self.speed_stop = rospy.get_param("~speed_stop", 50)
        self.distance_start = rospy.get_param("~distance_start", 1.2)
        self.distance_stop = rospy.get_param("~distance_stop", 2.4)

        # Publishers for the controllers
        # Controllers themselves spawned in the state machines respective launch files

        self.velocity_pub = rospy.Publisher(
            "/output/drive_velocity_controller/command", Float64, queue_size=10
        )
        self.steering_pub = rospy.Publisher(
            "/output/steering_position_controller/command", Float64, queue_size=10
        )
        self.vis_pub = rospy.Publisher(
            "/output/target_point",
            PointStamped,
        )

        # Diagnostics Publisher
        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        # Subscriber for path
        self.path_sub = rospy.Subscriber(
            "/input/path", Path, self.getPathplanningUpdate
        )
        self.odom_sub = rospy.Subscriber("/input/odom", Odometry, self.get_odom_update)

        self.current_pos = [0, 0]  # in blf
        self.current_path = np.zeros((0, 2))

        """
          Trajectory parameters and conditions
            - minimal_distance: the minimal required distance between the car and the candidate target point
            - max_angle: the maximal allowed angle difference between the car and the candidate target point
            - t_step: the t step the alg takes when progressing through the underlying parametric equations
                      Indirectly determines how many points are checked per segment.
        """
        self.minimal_distance = rospy.get_param("~trajectory/minimal_distance", 2)
        self.trajectory = Trajectory()
        self.publish_rate = rospy.get_param("~publish_rate", 10)

        self.speed_target = rospy.get_param("~speed/target", 3.0)
        self.steering_transmission = rospy.get_param(
            "ugr/car/steering/transmission", 0.25
        )  # Factor from actuator to steering angle

        # Helpers
        self.start_sender()

    def get_odom_update(self, msg: Odometry):
        self.actual_speed = msg.twist.twist.linear.x

    def getPathplanningUpdate(self, msg: Path):
        """
        Takes in a new exploration path coming from the mapping algorithm.
        The path should be relative to self.base_link_frame. Otherwise it will transform to it
        """

        # Transform received message to self.base_link_frame
        trans = self.tf_buffer.lookup_transform(
            self.base_link_frame,
            msg.header.frame_id,
            msg.header.stamp,
        )
        new_header = Header(frame_id=self.base_link_frame, stamp=trans.header.stamp)
        transformed_path = Path(header=new_header)
        for pose in msg.poses:
            pose_t = do_transform_pose(pose, trans)
            transformed_path.poses.append(pose_t)

        # Create a new path
        current_path = np.zeros((0, 2))
        for pose in transformed_path.poses:
            current_path = np.vstack(
                (current_path, [pose.pose.position.x, pose.pose.position.y])
            )

        # Save current path and time of transformation to self.trajectory
        self.trajectory.points = current_path
        self.trajectory.time_source = trans.header.stamp

    def symmetrically_bound_angle(self, angle, max_angle):
        """
        Helper function to bound {angle} to [-max_angle, max_angle]
        """
        return (angle + max_angle) % (2 * max_angle) - max_angle

    def start_sender(self):
        """
        Start sending updates. If the data is too old, brake.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            try:
                self.speed_target = rospy.get_param("~speed/target", 3.0)

                # First try to get a target point
                # Change the look-ahead distance (minimal_distance)  based on the current speed
                if self.actual_speed < self.speed_start:
                    self.minimal_distance = self.distance_start
                elif self.actual_speed < self.speed_stop:
                    self.minimal_distance = self.distance_start + (
                        self.distance_stop - self.distance_start
                    ) / (self.speed_stop - self.speed_start) * (
                        self.actual_speed - self.speed_start
                    )
                else:
                    self.minimal_distance = self.distance_stop

                # Calculate target point
                target_x, target_y, success = self.trajectory.calculate_target_point(
                    self.minimal_distance,
                )

                if not success:
                    # BRAKE! We don't know where to drive to!
                    rospy.loginfo("No target point found!")
                    self.diagnostics_pub.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.ERROR,
                            name="[CTRL PP] Target Point Status",
                            message="No target point found!",
                        )
                    )
                    self.velocity_cmd.data = 0.0
                    self.steering_cmd.data = 0.0
                else:
                    # Calculate required turning radius R and apply inverse bicycle model to get steering angle (approximated)
                    R = ((target_x - 0) ** 2 + (target_y - 0) ** 2) / (
                        2 * (target_y - 0)
                    )

                    self.steering_cmd.data = self.symmetrically_bound_angle(
                        np.arctan2(1.0, R), np.pi / 2
                    )

                    self.diagnostics_pub.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name="[CTRL PP] Target Point Status",
                            message="Target point found.",
                        )
                    )

                    # Go ahead and drive. But adjust speed in corners
                    self.velocity_cmd.data = self.speed_target

                # Publish to velocity and position steering controller
                self.steering_cmd.data /= self.steering_transmission
                self.steering_pub.publish(self.steering_cmd)

                self.velocity_cmd.data /= (
                    self.wheelradius
                )  # Velocity to angular velocity
                self.velocity_pub.publish(self.velocity_cmd)

                point = PointStamped()
                point.header.stamp = self.trajectory.time_source
                point.header.frame_id = self.base_link_frame
                point.point.x = target_x
                point.point.y = target_y
                self.vis_pub.publish(point)

            except Exception as e:
                rospy.logwarn(f"PurePursuit has caught an exception: {e}")
                import traceback

                print(traceback.format_exc())

            rate.sleep()


node = PurePursuit()
