#!/usr/bin/env python3

from collections import deque
from os import curdir
from sqlite3 import Time
import time

import numpy as np
from genpy import Duration
import rospy
from fastslam.fastslam import FastSLAM
from nav_msgs.msg import Odometry
from slam.helpers import observations_to_range_bearings
from slam.slam import SLAMNode
from ugr_msgs.msg import Observations
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from node_fixture.node_fixture import ROSNode


class LocalFastSLAM(SLAMNode):
    def __init__(self):
        super().__init__("local_fastslam")

        # self.do_time_transform = True

        # Extra fastslam1.0 parameters
        self.particle_count = rospy.get_param("~fastslam/particle_count", 100)
        self.meas_cov = rospy.get_param(
            "~fastslam/measurement_covariance", [0.05, 0, 0, 0.001]
        )
        self.input_noise = rospy.get_param(
            "~fastslam/input_noise", [0.2, 0.2, 0.1, 0.1, 0.01, 0.01]
        )

        self.publish_rate = rospy.get_param("~fastslam/publish_rate", 10)

        self.threshold_distance = rospy.get_param("~fastslam/threshold_distance", 2)

        self.fastslam = FastSLAM(
            np.array(self.meas_cov).reshape((2, 2)),
            np.array(self.input_noise).reshape((3, 2)),
            self.particle_count,
            self.threshold_distance,
            self.max_landmark_range,
            "position",
        )

        self.uque = deque([])

        self.busy = False
        self.previous_timestamp = 1

        self.yaw = 0

        self.previous_pose = [0, 0, 0]
        self.current_pose = None

        # self.timer = rospy.Timer(
        #     rospy.Duration(1 / self.publish_rate),
        #     self.process,
        # )

        self.observations = None

    def get_predictions(self):
        (
            state_prediction,
            map_prediction,
            landmark_classes,
            path_prediction,
            particle_states,
            particle_weigth
        ) = self.fastslam.get_prediction()
        return state_prediction, map_prediction, landmark_classes, path_prediction, particle_states, particle_weigth

    def process(self, timer):
        pass
        # try:
        #     # Timestamp for measuring dt
        #     timestamp = rospy.Time.now()
        #     timestamp_sec = timestamp.to_sec()

        #     # Check if there are any observations
        #     obs_rb = []
        #     if self.observations:

        #         observations = self.observations

        #         # Now do a "time transformation" to keep delays in mind
        #         transform: TransformStamped = self.tf_buffer.lookup_transform_full(
        #             self.base_link_frame,
        #             timestamp,
        #             self.base_link_frame,
        #             observations.header.stamp,
        #             self.world_frame,  # Needs a fixed frame to use as fixture for the transformation
        #             rospy.Duration(1)
        #         )
        #         time_transformed_observations = ROSNode.do_transform_observations(
        #             observations, transform
        #         )

        #         print(f"DIFFERENCE: {timestamp.to_sec() - observations.header.stamp.to_sec()} seconds")
                
        #         obs_rb = observations_to_range_bearings(time_transformed_observations)

        #         self.observations = None


        #     # Lookup the current position of the car to calculate what delta values to feed FastSLAM.
        #     # This position lookup should be the latest position as the primary goal of FastSLAM is using observations
        #     # to localize the car
        #     transform: TransformStamped = self.tf_buffer.lookup_transform(
        #         self.world_frame,
        #         self.base_link_frame,
        #         timestamp,
        #         rospy.Duration(0.01)
        #     )
        #     x = transform.transform.translation.x
        #     y = transform.transform.translation.y
        #     _, _, yaw = euler_from_quaternion(
        #         [
        #             transform.transform.rotation.x,
        #             transform.transform.rotation.y,
        #             transform.transform.rotation.z,
        #             transform.transform.rotation.w,
        #         ]
        #     )

        #     # Initialization guard
        #     if self.previous_timestamp == 0 or self.previous_pose is None:
        #         self.previous_pose = [x, y, yaw]
        #         self.previous_timestamp = timestamp_sec
        #         self.busy = False
        #         return

        #     # FastSLAM needs [dDistance, dtheta] and observations in polar coordinates
        #     x0, y0, yaw0 = self.previous_pose
        #     dDist = (  (x - x0) ** 2 + (y - y0) ** 2 ) ** (1/2)

        #     # Actually feed FastSLAM now
        #     self.fastslam.step(
        #         [dDist, yaw - yaw0],
        #         self.previous_timestamp - timestamp_sec, 
        #         obs_rb,
        #     )

        #     # Save for next iteration
        #     self.previous_pose = [x, y, yaw]
        #     self.previous_timestamp = timestamp_sec

        #     print(f"SLAM took {rospy.Time.now().to_sec() - timestamp_sec} seconds")

        #     self.busy = False

        #     self.publish_result(timestamp)


        # except Exception as e:
        #     rospy.logerr(e)
        #     self.busy = False


    def process_odometry(self, odometry: Odometry):
        pass

    def process_observations(self, observations: Observations):
        
        self.observations = observations

        if self.busy:
            rospy.logfatal(
                "It seems like there are multiple (active) publishers on the input observations topic. Local FastSLAM must process data sequentially. This means that there can only be one (active) publisher..."
            )
            return

        self.busy = True

        try:

            # Timestamp for measuring dt
            timestamp = rospy.Time.now()
            timestamp_sec = timestamp.to_sec()

            # Lookup the current position of the car to calculate what delta values to feed FastSLAM.
            # This position lookup should be the latest position as the primary goal of FastSLAM is using observations
            # to localize the car
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_link_frame,
                observations.header.stamp,
                rospy.Duration(0.1)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            _, _, yaw = euler_from_quaternion(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            )

            # Initialization guard
            if self.previous_timestamp == 0 or self.previous_pose is None:
                self.previous_pose = [x, y, yaw]
                self.previous_timestamp = timestamp_sec
                self.busy = False
                return

            # FastSLAM needs [dDistance, dtheta] and observations in polar coordinates
            obs_rb = observations_to_range_bearings(observations)
            x0, y0, yaw0 = self.previous_pose
            dDist = (  (x - x0) ** 2 + (y - y0) ** 2 ) ** (1/2)

            # Actually feed FastSLAM now
            self.fastslam.step(
                [dDist, yaw - yaw0],
                self.previous_timestamp - timestamp_sec, 
                obs_rb,
            )

            # Save for next iteration
            self.previous_pose = [x, y, yaw]
            self.previous_timestamp = timestamp_sec

            print(f"SLAM took {rospy.Time.now().to_sec() - timestamp_sec} seconds")

            self.busy = False

            self.publish_result(timestamp)


        except Exception as e:
            rospy.logerr(e)
            self.busy = False


node = LocalFastSLAM()
node.start()
