#!/usr/bin/env python3

# import time
import os
from pathlib import Path

import cv2 as cv
import numpy as np

# import numpy.typing as npt
import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from ugr_msgs.msg import BoundingBoxesStamped


class EarlyFusion:
    def __init__(self):
        rospy.init_node("early_fusion")
        self.pc_header = None
        self.pc = None
        self.bbox_header = None
        self.bboxes = None
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
        self.orig_translation_vector = np.array(
            [-0.09, 0, -0.40]
        )  # original [-0.09, 0, -0.37]

    def handle_pc(self, msg: PointCloud2):
        self.pc_header = msg.header  # stamp, frame_id
        # rospy.loginfo("Got pointcloud")
        self.pc = np.array(
            list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        )

    def handle_bbox(self, msg: BoundingBoxesStamped):
        self.bbox_header = msg.header
        self.bboxes = msg.bounding_boxes  # for now just work with latest boxes
        # rospy.loginfo("Got bboxes")
        # print(len(self.bboxes))
        count = 0
        print("start")
        if self.pc is not None:
            # ratios = np.abs(self.pc[:, 1] / self.pc[:, 0])
            # self.pc = self.pc[ratios < self.max_ratio]
            transformed = self.transform_points(self.pc)
            # for p in transformed:
            #     if p[0][0] > 0 and p[0][0] < 1920 and p[0][1] > 0 and p[0][1] < 1200:
            #         print(list(p[0]))
            #         print(",")
            # print(transformed[:,0])

            total = []
            for box in self.bboxes:
                inside_pts = []
                distances = []
                for i, point in enumerate(transformed):
                    if (
                        point[0][0] > box.left
                        and point[0][0] < box.left + box.width
                        and point[0][1] > box.top
                        and point[0][1] < box.top + box.height
                    ):
                        # print(point[0])
                        inside_pts.append(self.pc[i])
                        distances.append(np.linalg.norm(self.pc[i]))
                        total.append(list(point[0]))

                if inside_pts == []:
                    continue
                count += 1
                closest_distance = min(distances)
                closest_pt = inside_pts[distances.index(closest_distance)]
                filtered_pts = []
                for point in inside_pts:
                    if np.linalg.norm(point - closest_pt) < 0.5:
                        filtered_pts.append(point)
            # print(total)
            self.pc = None

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
