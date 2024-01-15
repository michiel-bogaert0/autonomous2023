#!/usr/bin/env python3
import rospy
import tf2_ros as tf
from nav_msgs.msg import Odometry, Path
from node_fixture.fixture import DiagnosticArray, NodeManagingStatesEnum, ROSNode
from node_fixture.node_management import ManagedNode
from std_msgs.msg import Float64

from .trajectory import Trajectory


class KinematicTrackingNode(ManagedNode):
    def __init__(self, name):
        rospy.init_node(name)
        super().__init__(name)

        self.publish_rate = rospy.get_param("~publish_rate", 10)

        # Publishers for the controllers
        # Controllers themselves spawned in the state machines respective launch files

        self.velocity_pub = super().AddPublisher(
            "/output/drive_velocity_controller/command", Float64, queue_size=10
        )
        self.steering_pub = super().AddPublisher(
            "/output/steering_position_controller/command", Float64, queue_size=10
        )

        # Diagnostics Publisher
        self.diagnostics_pub = super().AddPublisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )

        # Subscriber for path
        self.path_sub = super().AddSubscriber(
            "/input/path", Path, self.getPathplanningUpdate
        )

        self.odom_sub = super().AddSubscriber(
            "/input/odom", Odometry, self.get_odom_update
        )

    def doConfigure(self):
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.wheelradius = rospy.get_param("~wheelradius", 0.1)

        self.velocity_cmd = Float64(0.0)
        self.steering_cmd = Float64(0.0)
        self.actual_speed = 0.0

        self.received_path = None

        self.speed_target = rospy.get_param("/speed/target", 3.0)
        self.steering_transmission = rospy.get_param(
            "ugr/car/steering/transmission", 0.25
        )  # Factor from actuator to steering angle

    def doActivate(self):
        # Do this here because some parameters are set in the mission yaml files
        self.trajectory = Trajectory()

    def get_odom_update(self, msg: Odometry):
        self.actual_speed = msg.twist.twist.linear.x

    def getPathplanningUpdate(self, msg: Path):
        """
        Takes in a new exploration path coming from the mapping algorithm.
        Does not do anything with it, unless the control loop is active
        """
        if msg is None:
            return

        self.received_path = msg

    def doUpdate(self):
        """
        Actually processes a new path
        The path should be relative to self.base_link_frame. Otherwise it will transform to it
        """

        if self.received_path is None:
            return

        msg = self.received_path

        # Transform received message to self.base_link_frame
        trans = self.tf_buffer.lookup_transform(
            self.base_link_frame, msg.header.frame_id, msg.header.stamp
        )
        transformed_path = ROSNode.do_transform_path(msg, trans)

        # Save current path and time of transformation to self.trajectory
        self.trajectory.set_path(transformed_path)
        self.received_path = None

    def symmetrically_bound_angle(self, angle, max_angle):
        """
        Helper function to bound {angle} to [-max_angle, max_angle]
        """
        return (angle + max_angle) % (2 * max_angle) - max_angle

    def __process__(self):
        raise NotImplementedError

    def start_sender(self):
        """
        Start sending updates. If the data is too old, brake.
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.state == NodeManagingStatesEnum.ACTIVE:
                try:
                    # First fetch new paths, then process current path
                    self.doUpdate()
                    self.__process__()

                except Exception as e:
                    rospy.logwarn(f"PurePursuit has caught an exception: {e}")
                    import traceback

                    print(traceback.format_exc())

            rate.sleep()
