import sys
from typing import Tuple
import rospy
import numpy as np
from triangulation.center_points import get_center_points, filter_center_points
from triangulation.paths import TriangulationPaths
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from std_msgs.msg import Header


class Triangulator:
    def __init__(
        self,
        triangulation_min_var: float,
        triangulation_var_threshold: float,
        max_iter: int,
        max_angle_change: float,
        max_path_distance: float,
        safety_dist: float,
        sorting_range: float,
        vis_points: rospy.Publisher=None,
        vis_lines: rospy.Publisher=None,
        vis_namespace: str="pathplanning_vis",
        vis_lifetime: float=0.2,
    ) -> None:
        """Initialize triangulator

        Args:
            triangulation_min_var: the maximum variance each allowed set of triangle edge lengths can always have.
                So it's the minimal maximum variance
            triangulation_var_threshold: Factor multiplied to the median of the variance of triangle lengths
                in order to filter bad triangles
            max_iter: Amount of iterations
            max_angle_change: Maximum angle change in path
            max_path_distance: Maximum distance between nodes in the planned path
            safety_dist: Safety distance from objects
            vis_points: (optional) rostopic for publishing MarkerArrays
            vis_lines: (optional) rostopic for publishing PoseArrays
            vis_namespace: (optional) namespace for publishing markers
            vis_lifetime: (optional) visualisation marker lifetime
        """
        self.triangulation_min_var = triangulation_min_var
        self.triangulation_var_threshold = triangulation_var_threshold
        self.max_iter = max_iter
        self.max_angle_change = max_angle_change
        self.max_path_distance = max_path_distance
        self.safety_dist = safety_dist
        self.safety_dist_squared = safety_dist**2
        self.sorting_range = sorting_range

        # `vis` is only enabled if both point and line visualisation are passed through
        self.vis = vis_points is not None and vis_lines is not None
        self.vis_points = vis_points
        self.vis_lines = vis_lines
        self.vis_namespace = vis_namespace
        self.vis_lifetime = vis_lifetime

        self.triangulation_paths = TriangulationPaths(
            max_iter, max_angle_change, max_path_distance, safety_dist
        )

    def get_path(self, cones: np.ndarray, header: Header):
        """Generate path based on the cones.

        Args:
            cones: Cones between which to generate path (array of [x, y, class])
            header: the message header of the original observations

        Returns:
            path
        """
        position_cones = cones[:, :-1]  # Extract the x and y positions

        # We need at least 4 cones for Delaunay triangulation
        if len(position_cones) < 4:
            print("Not enough cones for triangulation.")
            return None

        # Perform triangulation and get the (useful) center points
        triangulation_centers, center_points, triangles = get_center_points(
            position_cones, self.triangulation_min_var, self.triangulation_var_threshold, self.sorting_range
        )

        # Try to get rid of false positive
        center_points = filter_center_points(
            center_points, triangulation_centers, cones
        )

        # Publish visualisation topics if needed
        if self.vis:
            self.publish_points(
                position_cones,
                header,
                self.vis_namespace + "/cones",
                (1, 1, 0, 1),
                0.3,
            )
            self.publish_points(
                triangulation_centers,
                header,
                self.vis_namespace + "/triangulation_centers",
                (1, 0, 0, 1),
            )
            self.publish_points(
                center_points,
                header,
                self.vis_namespace + "/center_points",
                (0, 1, 0, 1),
                0.2,
            )
            self.visualise_triangles(triangles, header)

        _, leaves = self.triangulation_paths.get_all_paths(
            triangulation_centers, center_points, cones[:, :-1], self.sorting_range
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
            ) = self.triangulation_paths.get_cost_branch(path, cones, self.sorting_range)
            costs[i] = [
                angle_cost,
                length_cost,
            ]
            paths.append(path)
            path_lengths = np.append(path_lengths, len(path))
        
        costs = costs / np.max(costs, axis=0) #normalize costs
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
