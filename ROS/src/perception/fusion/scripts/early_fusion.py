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
        self.covariance = rospy.get_param(
            "~covariance", [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.8]
        )
        self.belief = rospy.get_param("~belief", 0.8)
        self.cone_width = rospy.get_param("~cone_width", 0.232)
        self.rotation_matrix = np.array(
            [
                [0.9998479, 0.038, 0.0000000],
                [-0.038, 0.9998479, 0.03],
                [0.0000000, -0.03, 1.0000000],
            ]
        )
        self.translation_vector = np.array(
            rospy.get_param("~translation_vector", [-0.09, 0, -0.40])
        )
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

        rospy.Subscriber("/input/lidar_groundremoval", PointCloud2, self.handle_pc)
        rospy.Subscriber(
            "/input/camera_bboxes",
            BoundingBoxesStamped,
            self.handle_bbox,
        )
        self.result_publisher = rospy.Publisher(
            "/output/topic", ObservationWithCovarianceArrayStamped, queue_size=10
        )

    def publish(self, msg):
        """
        Just publishes on the topic
        """
        self.result_publisher.publish(msg)

    def handle_pc(self, msg: PointCloud2):
        """
        Callback for the lidar point cloud topic. Stores the point cloud data and sets a flag indicating new data is available.
        """
        # rospy.loginfo("Got pointcloud")
        self.pc_header = msg.header
        self.pc = np.array(
            list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        )

    def handle_bbox(self, msg: BoundingBoxesStamped):
        """
        Callback for the bounding boxes topic. Stores the bounding box data and runs the fusion process.
        """
        # rospy.loginfo("Got bboxes")
        self.bbox_header = msg.header
        self.bboxes = msg.bounding_boxes
        if self.pc is not None:
            self.run_fusion()

    def run_fusion(self):
        """
        Performs the fusion process. Transforms the point cloud data, filters points inside each bounding box,
        calculates the centroid of the filtered points, and publishes the results.
        """
        pointcloud = np.copy(self.pc)
        self.pc = None  # the same pc cannot be used twice (pc's are received at almost double the rate of bboxes)
        projections = self.project_points(pointcloud)
        observations = []
        for box in self.bboxes:
            inside_pts = self.filter_points_inside_bbox(pointcloud, projections, box)
            if inside_pts == []:
                continue
            centroid = self.calculate_centroid(inside_pts)
            if centroid is None:
                continue
            observations.append(self.create_observation(centroid, box.cone_type))
        results = self.create_results(observations)
        self.publish(results)

    def create_results(self, observations):
        """
        Creates an ObservationWithCovarianceArrayStamped message from a list of observations
        """
        results = ObservationWithCovarianceArrayStamped()
        results.observations = observations
        results.header.stamp = (
            self.bbox_header.stamp
        )  # take the timestamp from the latest bounding box message
        results.header.frame_id = self.pc_header.frame_id  # take the lidar frame id
        return results

    def calculate_centroid(self, points):
        """
        Calculates the centroid of a set of points
        """
        mean_point = np.median(
            points, axis=0
        )  # calculate the median point, this should always be a point very close to the cone
        filtered_pts = [
            point for point in points if np.linalg.norm(point - mean_point) < 0.5
        ]  # filter out points that are too far from the mean
        if filtered_pts == []:
            return None
        centroid = np.mean(filtered_pts, axis=0)
        direction_vector = centroid / np.linalg.norm(centroid)
        centroid += direction_vector * (
            1 / 6 * self.cone_width
        )  # compensation for only seeing the front of the cone
        return centroid

    def filter_points_inside_bbox(self, pc, projections, bbox):
        """
        Filters the projected points inside the bounding box
        """
        inside_pts = []
        for i, projection in enumerate(projections):
            if (
                projection[0][0] > bbox.left
                and projection[0][0] < bbox.left + bbox.width
                and projection[0][1] > bbox.top
                and projection[0][1] < bbox.top + bbox.height
            ):
                inside_pts.append(pc[i])
        return inside_pts

    def create_observation(self, centroid, cone_type):
        """
        Creates an ObservationWithCovariance message from a centroid and color
        """
        cone = ObservationWithCovariance()
        cone.observation.location.x = centroid[0]
        cone.observation.location.y = centroid[1]
        cone.observation.location.z = centroid[2]
        cone.covariance = self.covariance
        cone.observation.observation_class = cone_type
        cone.observation.belief = self.belief
        return cone

    def project_points(self, points):
        """
        Transforms the 3D point cloud data from the lidar frame to the 2D camera frame
        """
        transformed_points = np.copy(points)
        points += self.translation_vector
        points = points @ self.rotation_matrix

        # change the axis to match the opencv coordinate system
        transformed_points[:, 0] = -points[:, 1]
        transformed_points[:, 1] = -points[:, 2]
        transformed_points[:, 2] = points[:, 0]
        projections = cv.projectPoints(
            transformed_points,
            np.array([0, 0, 0], dtype=np.float32),
            np.array([0, 0, 0], dtype=np.float32),
            self.camera_matrix,
            np.zeros((1, 5)),
        )[0]
        return projections


node = EarlyFusion()
rospy.spin()
