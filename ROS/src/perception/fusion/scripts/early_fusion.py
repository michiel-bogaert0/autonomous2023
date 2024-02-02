#!/usr/bin/env python3

import os
from pathlib import Path

import cv2 as cv
import numpy as np
import rospy
import tf2_ros as tf
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from ugr_msgs.msg import (
    BoundingBoxesStamped,
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
)


class EarlyFusion:
    def __init__(self):
        rospy.init_node("early_fusion")
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.pc_header = None
        self.pc = None
        self.bbox_header = None
        self.bboxes = None
        self.cone_width = 0.232
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        camcal_location = rospy.get_param(
            "/perception/camera/camcal_location", "camera_calibration_baumer.npz"
        )
        camera_cal_archive = np.load(
            Path(os.getenv("BINARY_LOCATION")) / "pnp" / camcal_location
        )

        self.camera_matrix = camera_cal_archive["camera_matrix"]
        self.distortion_matrix = camera_cal_archive["distortion_matrix"]

        self.orig_rotation_matrix = np.array(
            [
                [0.9998479, -0.038, 0.0000000],
                [0.038, 0.9998479, -0.03],
                [0.0000000, 0.03, 1.0000000],
            ]
        )

        rospy.Subscriber("/input/lidar_groundremoval", PointCloud2, self.handle_pc)
        rospy.Subscriber(
            "/input/camera_bboxes",
            BoundingBoxesStamped,
            self.handle_bbox,
        )
        self.result_publisher = rospy.Publisher(
            "/output/topic", ObservationWithCovarianceArrayStamped, queue_size=10
        )

        self.orig_translation_vector = np.array(
            [-0.09, 0, -0.40]
        )  # original [-0.09, 0, -0.37]

    def publish(self, msg):
        """
        Just publishes on the topic
        """
        self.result_publisher.publish(msg)

    def handle_pc(self, msg: PointCloud2):
        self.pc_header = msg.header
        rospy.loginfo("Got pointcloud")
        self.pc = np.array(
            list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        )

    def handle_bbox(self, msg: BoundingBoxesStamped):
        self.bbox_header = msg.header
        self.bboxes = msg.bounding_boxes
        rospy.loginfo("Got bboxes")
        if self.pc is not None:
            self.run_fusion()

    def run_fusion(self):
        transformed = self.transform_points(self.pc)
        results = ObservationWithCovarianceArrayStamped()
        used_points = []
        for box in self.bboxes:
            inside_pts = []
            distances = []
            indices = []
            for i, point in enumerate(transformed):
                if (
                    i not in used_points
                    and point[0][0] > box.left
                    and point[0][0] < box.left + box.width
                    and point[0][1] > box.top
                    and point[0][1] < box.top + box.height
                ):
                    inside_pts.append(self.pc[i])
                    distances.append(np.linalg.norm(self.pc[i]))
                    indices.append(i)

            if inside_pts == []:
                continue
            closest_distance = min(distances)
            closest_pt = inside_pts[distances.index(closest_distance)]
            filtered_pts = []
            for i, point in enumerate(inside_pts):
                if np.linalg.norm(point - closest_pt) < 0.5:
                    filtered_pts.append(point)
                    used_points.append(indices[i])
            if not filtered_pts:
                continue
            centroid = np.mean(filtered_pts, axis=0)
            direction_vector = centroid / np.linalg.norm(centroid)
            centroid += direction_vector * (1 / 6 * self.cone_width)
            cone = ObservationWithCovariance()
            cone.observation.location.x = centroid[0]
            cone.observation.location.y = centroid[1]
            cone.observation.location.z = centroid[2]
            cone.covariance = [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.8]
            cone.observation.observation_class = box.cone_type
            cone.observation.belief = 0.8
            results.observations.append(cone)
        self.pc = None
        results.header.stamp = self.bbox_header.stamp
        results.header.frame_id = self.pc_header.frame_id
        self.publish(results)

    def transform_points(self, points):
        points += self.orig_translation_vector
        points = points @ self.orig_rotation_matrix.T

        transformed_points = np.copy(points)

        transformed_points[:, 0] = -points[:, 1]
        transformed_points[:, 1] = -points[:, 2]
        transformed_points[:, 2] = points[:, 0]
        transformed_points = cv.projectPoints(
            transformed_points,
            np.array([0, 0, 0], dtype=np.float32),
            np.array([0, 0, 0], dtype=np.float32),
            self.camera_matrix,
            np.zeros((1, 5)),
        )[0]
        return transformed_points


node = EarlyFusion()
rospy.spin()
