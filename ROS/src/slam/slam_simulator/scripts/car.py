#!/usr/bin/env python3
import rospy
import numpy as np

from pynput import keyboard
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    TwistWithCovarianceStamped,
    Quaternion,
    TransformStamped
)
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
import tf2_ros as tf2

import models

class CarSimulator:
    def __init__(self) -> None:

        rospy.init_node("car_simulator")

        # Internal state
        self.driving_intention = 0
        self.steering_intention = 0

        self.key_state = {
            "up": False,
            "down": False,
            "right": False,
            "left": False
        }

        self.car_state = (0, 0, 0) # (x, y, heading)
        self.sensors = (0, 0, 0) # (forward velocity, linear acceleration, angular acceleration)

        # Parameters
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.gt_base_link_frame = rospy.get_param("~gt_base_link_frame", "ugr/gt_base_link")

        self.gt_publish_rate = rospy.get_param("~publish_rates/gt", 200)
        self.encoder_publish_rate = rospy.get_param("~publish_rates/encoder", 30)
        self.imu_publish_rate = rospy.get_param("~publish_rates/imu", 90)

        self.encoder_noise = rospy.get_param("~noise/encoder", [0, 0.05, 0.05])
        self.imu_acceleration_noise = rospy.get_param("~noise/imu_acceleration", [0.0, 0.1, 0.01])
        self.imu_angular_velocity_noise = rospy.get_param("~noise/imu_angular_velocity", [0, 0.1, 0.01])

        self.model_name = rospy.get_param("~model", "bicycle")

        if self.model_name == "bicycle":
            self.model =  models.BicycleModel()
        
        else:
            raise "Unknown model!"
        
        self.model.reset()

        # Input handler
        self.listener = keyboard.Listener()
        self.listener.start()
        self.key_events = keyboard.Events()

        # Publishers
        self.br = tf2.TransformBroadcaster()
        self.gt_pub = rospy.Publisher('/output/gt_odometry', Odometry, queue_size=5)
        self.encoder_pub = rospy.Publisher('/output/encoder0', TwistWithCovarianceStamped, queue_size=5)
        self.imu_pub = rospy.Publisher('/output/imu0', Imu, queue_size=5)

        # Set up publisher rates
        rospy.Timer(
            rospy.Duration(1 / self.gt_publish_rate),
            self.publish_gt,
        )
        rospy.Timer(
            rospy.Duration(1 / self.encoder_publish_rate),
            self.publish_encoder,
        )
        rospy.Timer(
            rospy.Duration(1 / self.imu_publish_rate),
            self.publish_imu,
        )

        print("Started! Press the arrow keys to start moving!")

        # Main loop
        try:
            with keyboard.Events() as events:

                t0 = rospy.Time.now().to_sec()
                while not rospy.is_shutdown():
                    self.key_events = events
                    self.get_input()
                    t1 = rospy.Time.now().to_sec()
                    self.car_state, self.sensors = self.model.update(t1 - t0, self.driving_intention, self.steering_intention)
                    t0 = t1

        except Exception as e:
            print(e)
        finally:
            pass

    def get_input(self):
        """
        Listens to an key press/release event and updates inputs accordingly
        """
        event = self.key_events.get(0.01)

        if event is not None:

            key = event.key
            if key == keyboard.Key.up:
                self.key_state["up"] = True if type(event) == keyboard.Events.Press else False
            elif key == keyboard.Key.down:
                self.key_state["down"] = True if type(event) == keyboard.Events.Press else False
            elif key == keyboard.Key.left:
                self.key_state["left"] = True if type(event) == keyboard.Events.Press else False
            elif key == keyboard.Key.right:
                self.key_state["right"] = True if type(event) == keyboard.Events.Press else False
            elif key == keyboard.Key.space:
                self.key_state = {
                    "up": False,
                    "down": False,
                    "right": False,
                    "left": False
                }
                self.model.stop()
            elif key == keyboard.Key.esc:
                self.key_state = {
                    "up": False,
                    "down": False,
                    "right": False,
                    "left": False
                }
                self.model.reset()
        
        if self.key_state["up"]:
            self.driving_intention = 1.0
        elif self.key_state["down"]:
            self.driving_intention = -1.0
        else:
            self.driving_intention = 0
        
        if self.key_state["right"]:
            self.steering_intention = -1.0
        elif self.key_state["left"]:
            self.steering_intention = 1.0
        else:
            self.steering_intention = 0.0

    def publish_gt(self, _):
        """
        Publishes GT Odometry message
        """

        x, y, theta = self.car_state
        v, _, omega = self.sensors

        # Ground truth odometry
        odom = Odometry()
        odom.header.stamp = rospy.Time().now()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.gt_base_link_frame
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation = Quaternion(
            quat[0], quat[1], quat[2], quat[3]
        )

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.gt_pub.publish(odom)

        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.gt_base_link_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)


    def apply_noise_and_quantise(self, x, noise):
        """
        Applies normal noise and linearly quantises the value x

        Args:
            x: value to 'noisify'
            noise: list of three elements [mean, standard deviation, quantisation step]
        """
        noisy_x = x + np.random.normal(noise[0], noise[1])
        noisy_x = round(noisy_x / noise[2]) * noise[2]
        return noisy_x

    def publish_encoder(self, _):
        """
        Publishes simulated encoder (TwistWithCovarianceStamped)
        """

        v, _, _ = self.sensors

        noisy_v = self.apply_noise_and_quantise(v, self.encoder_noise)

        twist = TwistWithCovarianceStamped()
        twist.header.stamp = rospy.Time().now()
        twist.header.frame_id = self.base_link_frame
        twist.twist.twist.linear.x = noisy_v

        self.encoder_pub.publish(twist)

    def publish_imu(self, _):
        """
        Publishes simulated IMU (only angular velocity) (Imu)
        """

        _, a, omega = self.sensors

        noisy_omega = self.apply_noise_and_quantise(omega, self.imu_angular_velocity_noise)
        noisy_a = self.apply_noise_and_quantise(a, self.imu_acceleration_noise)

        imu = Imu()
        imu.header.stamp = rospy.Time().now()
        imu.header.frame_id = self.base_link_frame
        imu.linear_acceleration.x = noisy_a
        imu.angular_velocity.z = noisy_omega

        self.imu_pub.publish(imu)


node = CarSimulator()
