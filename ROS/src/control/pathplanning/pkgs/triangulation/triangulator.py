import sys
from typing import Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from std_msgs.msg import Header
from triangulation.center_points import get_center_points
from triangulation.paths import TriangulationPaths
from visualization_msgs.msg import Marker, MarkerArray


class Triangulator:
    def __init__(
        self,
        triangulation_max_var: float,
        triangulation_var_threshold: float,
        max_iter: int,
        max_angle_change: float,
        max_path_distance: float,
        safety_dist: float,
        continuous_dist: float,
        stage1_rectangle_width: float,
        stage1_bad_points_threshold: int,
        stage1_center_points_threshold: int,
        stage2_rectangle_width: float,
        stage2_bad_points_threshold: int,
        stage2_center_points_threshold: int,
        max_depth: int,
        range_front: float,
        range_behind: float,
        range_sides: float,
        vis_points: rospy.Publisher = None,
        vis_lines: rospy.Publisher = None,
        vis_namespace: str = "pathplanning_vis",
        vis_lifetime: float = 0.2,
    ) -> None:
        """Initialize triangulator

        Args:
            triangulation_max_var: the maximum variance each allowed set of triangle edge lengths can always have.
            triangulation_var_threshold: Factor multiplied to the median of the variance of triangle lengths
                in order to filter bad triangles
            max_iter: Amount of iterations
            max_angle_change: Maximum angle change in path
            max_path_distance: Maximum distance between nodes in the planned path
            safety_dist: Safety distance from objects
            continuous_dist: the max distance between nodes in stage 1
            stage1_rect_width: width of the rectangle for line expansion
            stage1_bad_points_threshold: threshold for the amount of bad points crossings allowed
            stage1_center_points_threshold: threshold for the amount of center point (not used) crossings allowed
            stage2_rect_width: width of the rectangle for line expansion
            stage2_bad_points_threshold: threshold for the amount of bad points crossings allowed
            stage2_center_points_threshold: threshold for the amount of center point (not used) crossings allowed
            max_depth: maximal depth for path searching in stage 2
            range_front: The range in front of the car where cones should be kept
            range_behind: The range behind the car where cones should be kept
            range_sides: The range to the sides of the car where cones should be kept
            vis_points: (optional) rostopic for publishing MarkerArrays
            vis_lines: (optional) rostopic for publishing PoseArrays
            vis_namespace: (optional) namespace for publishing markers
            vis_lifetime: (optional) visualisation marker lifetime
        """
        self.triangulation_max_var = triangulation_max_var
        self.triangulation_var_threshold = triangulation_var_threshold
        self.max_iter = max_iter
        self.max_angle_change = max_angle_change
        self.max_path_distance = max_path_distance
        self.safety_dist = safety_dist
        self.safety_dist_squared = safety_dist**2
        self.range_front = range_front
        self.range_behind = range_behind
        self.range_sides = range_sides

        # `vis` is only enabled if both point and line visualisation are passed through
        self.vis = vis_points is not None and vis_lines is not None
        self.vis_points = vis_points
        self.vis_lines = vis_lines
        self.vis_namespace = vis_namespace
        self.vis_lifetime = vis_lifetime

        self.triangulation_paths = TriangulationPaths(
            max_iter,
            max_angle_change,
            max_path_distance,
            safety_dist,
            continuous_dist,
            stage1_rectangle_width,
            stage1_bad_points_threshold,
            stage1_center_points_threshold,
            stage2_rectangle_width,
            stage2_bad_points_threshold,
            stage2_center_points_threshold,
            max_depth,
        )

    def get_path(self, cones: np.ndarray, header: Header):
        """Generate path based on the cones.

        Args:
            cones: Cones between which to generate path (array of [x, y, class])
            header: the message header of the original observations

        Returns:
            path
        """
        cones = cones[
            cones[:, 2] <= 1
        ]  # Only keep blue and yellow cones as orange cones are irrelevant

        position_cones = cones[:, :-1]

        # We need at least 4 cones for Delaunay triangulation
        tries = 0
        while len(position_cones) < 4:
            if tries >= 3:
                return None

            rospy.loginfo(
                "Not enough cones for triangulation. Trying again with a larger rectangle"
            )

            # Make a larger rectangle to hopefully get more cones
            self.range_behind *= 2
            self.range_sides *= 2
            self.range_front *= 2

            position_cones = cones[:, :-1]  # Extract the x and y positions

            # Only keep cones within a rectangle around the car
            position_cones = position_cones[position_cones[:, 0] <= self.range_behind]
            position_cones = position_cones[
                abs(position_cones[:, 1]) <= self.range_sides
            ]
            position_cones = position_cones[position_cones[:, 0] <= self.range_front]

            tries += 1

        # Perform triangulation and get the (useful) center points
        triangulation_centers, center_points, triangles, bad_points = get_center_points(
            position_cones,
            cones[:, -1],
            self.triangulation_max_var,
            self.triangulation_var_threshold,
            self.range_front,
        )

        # Publish visualisation topics if needed
        if self.vis:
            self.publish_points(
                position_cones[np.where(cones[:, -1] == 0)],
                header,
                self.vis_namespace + "/cones/0",
                (0, 0, 1, 1),
                0.3,
            )
            self.publish_points(
                position_cones[np.where(cones[:, -1] == 1)],
                header,
                self.vis_namespace + "/cones/1",
                (1, 1, 0, 1),
                0.3,
            )
            self.publish_points(
                center_points,
                header,
                self.vis_namespace + "/center_points",
                (0, 1, 0, 1),
                0.2,
            )
            self.publish_points(
                bad_points,
                header,
                self.vis_namespace + "/bad_points",
                (1, 0, 0, 1),
                0.2,
            )

        _, leaves = self.triangulation_paths.get_all_paths(
            bad_points, center_points, cones[:, :-1], self.range_front
        )
        if leaves is None:
            return None

        path = self.get_best_path(leaves, cones)

        return path

    def get_best_path(self, leaves: list, cones: np.ndarray):
        """Get best path based from all generated paths.

        Args:
            leaves: leaves from each path
            cones: cones to navigate

        Returns:
        Best path
        """
        if not leaves:
            print("There were no available paths!")
            sys.exit(1)

        costs = np.zeros((len(leaves), 2))
        paths = []
        path_lengths = np.zeros(1)

        # Iterate each leaf
        for i, leave in enumerate(leaves):
            # Find the path connecting this leaf to the root and reverse it
            path = []
            parent = leave
            while parent.parent is not None:
                path.append(parent)
                parent = parent.parent
            path.reverse()

            # Calculate the path cost
            (
                angle_cost,
                length_cost,
            ) = self.triangulation_paths.get_cost_branch(path, cones, self.range_front)
            costs[i] = [
                angle_cost,
                length_cost,
            ]
            paths.append(path)
            path_lengths = np.append(path_lengths, len(path))

        costs = costs / np.max(costs, axis=0)  # normalize costs
        total_cost = np.sum(costs, axis=1)

        # Get the least-cost path
        index = np.argmin(total_cost)

        return paths[index]

    def publish_points(
        self,
        points: np.ndarray,
        header: Header,
        namespace: str,
        color: Tuple,
        scale: float = 0.1,
    ) -> None:
        """Publishes points to the already set vis_points topic

        Args:
            points: Nx2 array of point locations (x and y)
            header: the message header of the original observations
            namespace: the namespace to publish the markers to
            color: a tuple of RGBA values (between 0 and 1)
            scale: (optional) the scale of the visualised markers
        """
        marker_array = MarkerArray()

        for i, (x, y) in enumerate(points.reshape(-1, 2)):
            marker = Marker()

            marker.header.frame_id = header.frame_id
            marker.header.stamp = header.stamp
            marker.ns = namespace

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.id = i

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y

            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale

            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            marker.lifetime = rospy.Duration(self.vis_lifetime)

            marker_array.markers.append(marker)

        self.vis_points.publish(marker_array)

    def publish_line(
        self,
        line: np.ndarray,
        header: Header,
    ) -> None:
        """Publishes a line to the already set vis_lines topic

        Args:
            line: Nx2 array of point locations (x and y)
            header: the message header of the original observations
        """

        poses: list(Pose) = []
        for _, point in enumerate(line):
            pose: Pose = Pose()
            position: Point = Point()
            orientation: Quaternion = Quaternion()

            # Fill position from path point
            position.x = point[0]
            position.y = point[1]
            position.z = 0

            # Fill orientation, to invalidate
            orientation.x = 0
            orientation.y = 0
            orientation.z = 0
            orientation.w = 0

            # Fill pose and add to array
            pose.position = position
            pose.orientation = orientation
            poses += [pose]

        output: PoseArray = PoseArray()
        output.header.frame_id = header.frame_id
        output.header.stamp = header.stamp
        output.poses = poses

        self.vis_lines.publish(output)

    def visualise_triangles(self, triangles: np.ndarray, header: Header) -> None:
        """Visualises triangles using the line visualiser

        Args:
            triangles: array of Nx3x2 containing all Delaunay triangles
            header: the message header of the original observations
        """

        # TODO This creates false lines
        self.publish_line(triangles.reshape(-1, 2), header)
