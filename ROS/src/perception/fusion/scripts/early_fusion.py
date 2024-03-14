#!/usr/bin/env python3

import os
from pathlib import Path

import cv2 as cv
import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_matrix
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
        self.max_distance = rospy.get_param(
            "~max_distance", 0.5
        )  # max distance for a point (to the mean) to be considered part of the cone
        self.max_angle = (
            rospy.get_param("~fov", "70") / 2
        )  # in degrees, half of the field of view of the camera
        self.max_ratio = np.tan(
            np.radians(self.max_angle)
        )  # points with a higher ratio (y/x) are outside the fov
        self.resolution_x = rospy.get_param("~resolution_x", 1920)
        self.resolution_y = rospy.get_param("~resolution_y", 1200)
        self.covariance = rospy.get_param(
            "~covariance", [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.8]
        )
        self.belief = rospy.get_param("~belief", 0.8)
        self.cone_width = rospy.get_param("~cone_width", 0.232)
        self.fusion_ready = False
        self.rotation_matrix = np.array(
            [
                [0.9998479, 0.026, -0.2],
                [-0.026, 0.9998479, -0.045],
                [0.2, 0.045, 1.0000000],
            ]
        )
        self.translation_vector = np.array(
            rospy.get_param("~translation_vector", [-0.09, 0, -0.42])
        )
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.sensor_frame = rospy.get_param("~sensor_frame", "os_sensor")
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

        rate = rospy.Rate(rospy.get_param("~rate", 20))
        while not rospy.is_shutdown():
            self.run()
            rate.sleep()

    def run(self):
        """
        Runs the fusion process
        """
        if self.fusion_ready:
            self.run_fusion()
            self.fusion_ready = False

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
        if self.bboxes is not None:
            self.fusion_ready = True

    def handle_bbox(self, msg: BoundingBoxesStamped):
        """
        Callback for the bounding boxes topic. Stores the bounding box data and runs the fusion process.
        """
        # rospy.loginfo("Got bboxes")
        self.bbox_header = msg.header
        self.bboxes = msg.bounding_boxes
        if self.pc is not None:
            self.fusion_ready = True

    def run_fusion(self):
        """
        Performs the fusion process. Transforms the point cloud data, filters points inside each bounding box,
        calculates the centroid of the filtered points, and publishes the results.
        """
        pointcloud = self.fov_filter(np.copy(self.pc))
        pc_header = self.pc_header
        bboxes = self.bboxes
        bbox_header = self.bbox_header
        self.pc = None  # the same pc cannot be used twice
        self.bboxes = None  # the same bboxes cannot be used twice
        pointcloud = self.transform_pc(pointcloud, pc_header, bbox_header)
        if pointcloud is None:
            return
        projections, pointcloud = self.project_pc(pointcloud)
        cones = []  # list of tuples (centroid, cone_type, close_points_count)
        for box in bboxes:
            inside_pts = self.filter_points_inside_bbox(pointcloud, projections, box)
            if len(inside_pts) == 0:
                continue
            centroid, close_points_count = self.calculate_centroid(inside_pts)
            if centroid is None:
                continue
            cones.append((centroid, box.cone_type, close_points_count))
        cones = self.remove_duplicate_cones(cones)
        results = self.create_results(cones, bbox_header.stamp)
        self.publish(results)

    def fov_filter(self, pc):
        """
        Filters the points that are within the field of view of the camera
        """
        return pc[np.abs(pc[:, 1] / pc[:, 0]) < self.max_ratio]

    def create_results(self, cones, stamp):
        """
        Creates an ObservationWithCovarianceArrayStamped message from a list of observations
        """
        observations = []
        for cone in cones:
            centroid, cone_type = cone[0], cone[1]
            observations.append(self.create_observation(centroid, cone_type))
        results = ObservationWithCovarianceArrayStamped()
        results.observations = observations
        results.header.stamp = (
            stamp  # take the timestamp from the latest bounding box message
        )
        results.header.frame_id = self.sensor_frame  # take the lidar frame id
        return results

    def calculate_centroid(self, points):
        """
        Calculates the centroid of a set of points and also returns the number of points close to the centroid
        """
        mean_point = np.median(
            points, axis=0
        )  # calculate the median point, this should always be a point very close to the cone
        filtered_pts = np.array(
            [
                point
                for point in points
                if np.linalg.norm(point - mean_point) < self.max_distance
            ]
        )  # filter out points that are too far from the mean
        if len(filtered_pts) == 0:
            return None, None
        centroid = np.mean(filtered_pts, axis=0)
        direction_vector = centroid / np.linalg.norm(centroid)
        centroid += direction_vector * (
            1 / 6 * self.cone_width
        )  # compensation for only seeing the front of the cone
        close_points_count = len(filtered_pts)  # count of points close to the centroid
        return centroid, close_points_count

    def filter_points_inside_bbox(self, pc, projections, bbox):
        """
        Filters the projected points inside the bounding box
        """
        pc = np.array(pc)
        projections = np.array(projections)

        mask = (
            (projections[:, 0, 0] > bbox.left)
            & (projections[:, 0, 0] < bbox.left + bbox.width)
            & (projections[:, 0, 1] > bbox.top)
            & (projections[:, 0, 1] < bbox.top + bbox.height)
        )

        inside_pts = pc[mask]

        return inside_pts

    def remove_duplicate_cones(self, cones):
        """
        Removes duplicate cones caused by overlapping bboxes by checking if the distance between two cones is less than the cone width
        """
        for i in range(len(cones)):
            for j in range(i + 1, len(cones)):
                if np.linalg.norm(cones[i][0] - cones[j][0]) < self.cone_width:
                    if (
                        cones[i][2] > cones[j][2]
                    ):  # keep the cone with the most close points (which will be the closest cone)
                        del cones[j]
                    else:
                        del cones[i]
                    return self.remove_duplicate_cones(cones)
        return cones

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

    def transform_pc(self, pc, pc_header, bbox_header):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform_full(
                target_frame=self.sensor_frame,
                target_time=bbox_header.stamp,
                source_frame=self.sensor_frame,
                source_time=pc_header.stamp,
                fixed_frame=self.world_frame,
                timeout=rospy.Duration(0.1),
            )
            # Convert the quaternion to a rotation matrix
            rotation_matrix = quaternion_matrix(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            )[:3, :3]
            pc += np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ]
            )  # translation first in tf
            pc = pc @ rotation_matrix  # rotation second
            return pc
        except Exception as e:
            rospy.logwarn(
                f"EarlyFusion has caught an exception during transformation: {e}"
            )
            return None

    def project_pc(self, pc):
        """
        Transforms the 3D point cloud data from the lidar frame to the 2D camera frame and filters out the points that are outside of the camera's frame
        """
        pc_copy = np.copy(pc)
        pc += self.translation_vector  # translation to camera frame
        pc = pc @ self.rotation_matrix  # rotation to camera frame
        transformed_pc = np.copy(
            pc
        )  # change the axis to match the opencv coordinate system
        transformed_pc[:, 0] = -pc[:, 1]
        transformed_pc[:, 1] = -pc[:, 2]
        transformed_pc[:, 2] = pc[:, 0]
        projections = cv.projectPoints(
            transformed_pc,
            np.array([0, 0, 0], dtype=np.float32),
            np.array([0, 0, 0], dtype=np.float32),
            self.camera_matrix,
            np.zeros((1, 5)),
        )[0]

        valid_mask = (
            (projections[:, 0, 0] >= 0)
            & (projections[:, 0, 0] <= self.resolution_x)
            & (projections[:, 0, 1] >= 0)
            & (projections[:, 0, 1] <= self.resolution_y)
        )  # extra check, most will have been filtered out by the fov filter

        projections = projections[valid_mask]
        pc_copy = pc_copy[valid_mask.squeeze()]
        return projections, pc_copy


node = EarlyFusion()
rospy.spin()
