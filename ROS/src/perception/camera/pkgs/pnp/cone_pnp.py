#! /usr/bin/python3

import os
from collections import deque
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
from fs_msgs.msg import Cone
from geometry_msgs.msg import Point
from ugr_msgs.msg import (
    ConeKeypoint,
    ConeKeypoints,
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class ConePnp:
    def __init__(
        self, cone_models, scale, max_distance, camera_matrix, distortion_matrix
    ):
        self.cone_models = cone_models
        self.scale = scale
        self.max_distance = max_distance
        self.camera_matrix = camera_matrix
        self.distortion_matrix = distortion_matrix

    def generate_perception_update(
        self, cone_keypoints_msg: ConeKeypoints, img_size: Tuple[int, int]
    ) -> ObservationWithCovarianceArrayStamped:
        """
        Receives a keypoint update message and applies a PnP algorithm to it.
        It returns a ObservationWithCovarianceArrayStamped.

        Args:
            cone_keypoints_msg: the message that contains the cone keypoints
            img_size: a tuple (W, H) of the original image size
        """
        cone_relative_positions: List[Cone] = []

        for cone_keypoint in cone_keypoints_msg.cone_keypoints:

            ret, relative_position, rvec = self.calculate_relative_position_of_cone(
                cone_keypoint, img_size
            )

            if not ret:
                continue

            x, y, z = tuple(relative_position * self.scale)
            loc = Point(z[0], -1 * x[0], y[0])

            distance = (x**2 + y**2 + z**2) ** (1 / 2)
            if distance >= self.max_distance:
                continue

            cone_relative_positions.append(
                ObservationWithCovariance(
                    observation_class=cone_keypoint.bb_info.cone_type,
                    location=loc,
                    covariance=[0.7, 0, 0, 0, 0.3, 0, 0, 0, 0.1],
                )
            )

        perception_observation = ObservationWithCovarianceArrayStamped()
        perception_observation.observations = cone_relative_positions
        perception_observation.header.stamp = cone_keypoints_msg.header.stamp
        perception_observation.header.frame_id = cone_keypoints_msg.header.frame_id

        return perception_observation

    def draw_cone(
        self, img, cone_keypoint: ConeKeypoint, img_size: Tuple[int, int], rvec, tvec
    ):
        """
        Draws a simple visualization on parameter 'img'. In the future this must be implemented in a custom vis node

        Args:
            img: the opencv image object to draw on.
            cone_keypoint: the set of keypoints of one detected cone (corresponds to the ConeKeypoint message)
            img_size: size of img in pixels (width, height)
            rvec: opencv rotation vector from transformation
            tvec: opencv translation vector form transformation

        Returns:
            The same img object but with the visualization drawn onto it
        """

        self.preprocess_keypoints(cone_keypoint, img_size)

        # Project something to the image plane
        cone_model = self.cone_models[f"cone_{cone_keypoint.bb_info.cone_type}"]

        cone_height = cone_model[0][1]
        cone_radius = cone_model[-1][0]

        projection, jacobian = cv2.projectPoints(
            np.float32(
                [
                    [0, 0, 0],
                    [cone_radius, 0, 0],
                    [0, cone_height, 0],
                    [0, 0, cone_radius],
                ]
            ),
            rvec,
            tvec,
            self.camera_matrix,
            self.distortion_matrix,
        )

        # Draw the damn thing
        # bottom = np.array([processed_keypoints[0][0], processed_keypoints[0][1] + cone_keypoint.bb_info.height], int)
        bottom = projection[0].astype(int)
        projection = projection.astype(int)
        img = cv2.line(
            img, tuple(bottom.ravel()), tuple(projection[1].ravel()), (255, 0, 0), 5
        )
        img = cv2.line(
            img, tuple(bottom.ravel()), tuple(projection[2].ravel()), (0, 0, 255), 5
        )
        img = cv2.line(
            img, tuple(bottom.ravel()), tuple(projection[3].ravel()), (0, 255, 0), 5
        )

        return img

    def preprocess_keypoints(
        self, cone_keypoint: ConeKeypoint, img_size: Tuple[int, int]
    ):
        """
        A helper function for processing keypoints on a cone.

        The keypoints in a ConeKeypoint message are defined as a set of numbers (x, y) between 0 and 1, which corresponds with the
        position relative to the bounding box and the size of the bounding box. This function converts this format to an absolute
        (x, y) position in pixels, relative to the top-left corner (0, 0) of the image.

        Args:
            cone_keypoint: the object that contains the keypoints of a cone detection (= ConeKeypoint message)
            img_size: the size of the image that corresponds to the cone_keypoints, (width, height)

        Returns:
            the processed keypoints in the same order, but now in absolute pixels.
        """

        # Preprocessing of keypoints
        processed_keypoints = np.empty((len(cone_keypoint.keypoints), 2), np.float32)
        bb = cone_keypoint.bb_info

        for i, relative_keypoint in enumerate(cone_keypoint.keypoints):
            processed_keypoints[i, 0] = (
                (relative_keypoint.x * bb.width) + bb.left
            ) * img_size[0]
            processed_keypoints[i, 1] = (
                (relative_keypoint.y * bb.height) + bb.top
            ) * img_size[1]

        return processed_keypoints

    def calculate_relative_position_of_cone(
        self, cone_keypoint: ConeKeypoint, img_size: Tuple[int, int]
    ):
        """
        Actually applies the PnP algorithm

        Args:
          cone_keypoint: the keypoints of a detected cone (= ConeKeypoint message)
          img_size: the size of the image which corresponds to the detection

        Returns:
          ret: True or False. Did the algorithm succeed?
          tvec: the opencv translation vector that transforms the camera frame to the 3D 'cone frame' and vice-versa. None when ret == False
          rvec: the opencv rotation vector that transforms the camera frame to the 3D 'cone frame' and vice-versa. None when ret == False
        """

        # Preprocessing of keypoints
        processed_keypoints = self.preprocess_keypoints(cone_keypoint, img_size)
        bb = cone_keypoint.bb_info

        ret, rvec, tvec = cv2.solvePnP(
            self.cone_models[f"cone_{bb.cone_type}"],
            processed_keypoints,
            self.camera_matrix,
            self.distortion_matrix,
        )

        if ret:
            return True, tvec, rvec

        return False, None, None
