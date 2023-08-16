#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped

from std_msgs.msg import Float64, Header
from tf2_geometry_msgs import do_transform_pose
from trajectory import Trajectory
import tf2_ros as tf


class PurePursuit:
    def __init__(self):

        rospy.init_node("pure_pursuit_control")

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        self.min_speed = rospy.get_param("~speed/min", 0.3)
        self.min_corner_speed = rospy.get_param("~speed/min_corner_speed", 0.7)
        self.wheelradius = rospy.get_param("~wheelradius", 0.1)

        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/car_odom")

        self.velocity_cmd = Float64(0.0)
        self.steering_cmd = Float64(0.0)

        # Publishers for the controllers
        # Controllers themselves spawned in the state machines respective launch files

        self.velocity_pub = rospy.Publisher(
            "/output/drive_velocity_controller/command", Float64, queue_size=10
        )
        self.steering_pub = rospy.Publisher(
            "/output/steering_position_controller/command", Float64, queue_size=10
        )
        self.vis_pub = rospy.Publisher(
            "/output/target_point", PointStamped, 
        )

        # Subscriber for path
        self.path_sub = rospy.Subscriber(
            "/input/path", PoseArray, self.getPathplanningUpdate
        )

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
        self.minimal_distance = rospy.get_param("~trajectory/minimal_distance", 2)
        self.trajectory = Trajectory()
        self.publish_rate = rospy.get_param("~publish_rate", 10)
        self.speed_target = rospy.get_param("~speed/target", 3.0)
        self.steering_transmission = rospy.get_param("ugr/car/steering/transmission", 0.25) # Factor from actuator to steering angle

        # Helpers
        self.start_sender()

    def getPathplanningUpdate(self, msg: PoseArray):
        """
        Takes in a new exploration path coming from the mapping algorithm.
        The path should be relative to self.world_frame. Otherwise it will transform to it
        """

        # Transform received message
        trans = self.tf_buffer.lookup_transform(
            msg.header.frame_id,
            self.base_link_frame,
            rospy.Time(),
        )

        # Create a new path
        current_path = np.zeros((0, 2))
        for pose in msg.poses:
            current_path = np.vstack(
                (current_path, [pose.position.x, pose.position.y])
            )

        self.trajectory.set_path(current_path, [trans.transform.translation.x,  trans.transform.translation.y])

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

                # Lookup position
                trans = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.base_link_frame,
                    rospy.Time(),
                )

                # First try to get a target point
                # The target point is given in the world frame
                target_x, target_y, success = self.trajectory.calculate_target_point(
                    min(self.minimal_distance * 3, max(self.minimal_distance / 3, self.minimal_distance * self.velocity_cmd.data)), [trans.transform.translation.x,  trans.transform.translation.y]
                )

                # Transform to base_link frame
                invtrans = self.tf_buffer.lookup_transform(
                    self.base_link_frame,
                    self.world_frame,
                    rospy.Time(),
                )
                target_pose = PoseStamped(header=Header(frame_id=self.base_link_frame, stamp=rospy.Time.now()))
                target_pose.pose.position.x = target_x
                target_pose.pose.position.y = target_y

                target_pose_t = do_transform_pose(target_pose, invtrans)

                target_x = target_pose_t.pose.position.x
                target_y = target_pose_t.pose.position.y

                if not success:
                    # BRAKE! We don't know where to drive to!
                    rospy.loginfo("No target point found!")
                    self.velocity_cmd.data = 0.0
                    self.steering_cmd.data = 0.0
                else:

                    # Calculate required turning radius R and apply inverse bicycle model to get steering angle (approximated)
                    R = (
                        (target_x - self.current_pos[0]) ** 2
                        + (target_y - self.current_pos[1]) ** 2
                    ) / (2 * (target_y - self.current_pos[1]))

                    if self.speed_target < 0.05:
                        self.steering_cmd.data = 0
                    else:
                        self.steering_cmd.data = self.symmetrically_bound_angle(
                            np.arctan2(1.0, R), np.pi / 2
                        )
                    rospy.loginfo(f"x: {target_x}, y: {target_y} R: {R}, steering angle {self.steering_cmd.data}")

                    # Go ahead and drive. But adjust speed in corners
                    self.velocity_cmd.data = self.speed_target

                # Publish to velocity and position steering controller
                self.steering_cmd.data /= self.steering_transmission
                self.steering_pub.publish(self.steering_cmd)

                self.velocity_cmd.data /= self.wheelradius  # Velocity to angular velocity
                self.velocity_pub.publish(self.velocity_cmd)

                point = PointStamped()
                point.header.stamp = rospy.Time.now()
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
