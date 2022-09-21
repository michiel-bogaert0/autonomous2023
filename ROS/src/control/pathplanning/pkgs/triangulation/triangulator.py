import sys
from typing import Tuple
import rospy
import numpy as np
from triangulation.center_points import get_center_points
from triangulation.paths import TriangulationPaths
import triangulation.utils
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header


class Triangulator:
    def __init__(
        self,
        triangulation_var_threshold: float,
        max_iter: int,
        plan_dist: float,
        max_angle_change: float,
        safety_dist: float,
        vis_points=None,
        vis_lines=None,
    ) -> None:
        """Initialize triangulator

        Args:
            triangulation_var_threshold: Factor multiplied to the median of the variance of triangle lengths in order to filter bad triangles
            max_iter: Amount of iterations
            plan_dist: Maximum distance to plan path
            max_angle_change: Maximum angle change in path
            safety_dist: Safety distance from objects
            vis_points: (optional) rostopic for publishing MarkerArrays
            vis_lines: (optional) rostopic for publishing PoseArrays
        """
        self.triangulation_var_threshold = triangulation_var_threshold
        self.max_iter = max_iter
        self.plan_dist = plan_dist
        self.max_angle_change = max_angle_change
        self.safety_dist = safety_dist
        self.safety_dist_squared = safety_dist**2

        # `vis` is only enabled if both point and line visualisation are passed through
        self.vis = vis_points is not None and vis_lines is not None
        self.vis_points = vis_points
        self.vis_lines = vis_lines
        self.vis_namespace = "pathplanning_vis"  # TODO add parameter
        self.vis_lifetime = 3  # TODO add parameter

        self.triangulation_paths = TriangulationPaths(
            max_iter, plan_dist, max_angle_change, safety_dist
        )

    def get_path(self, cones: np.ndarray, frame_id: str):
        """Generate path based on the cones.

        Args:
            cones: Cones between which to generate path (array of [x, y, class])

        Returns:
            path
        """
        position_cones = cones[:, :-1]  # Extract the x and y positions

        # We need at least 4 cones for Delaunay triangulation
        if len(position_cones) < 4:
            return None

        # Perform triangulation and get the (useful) center points
        triangulation_centers, center_points = get_center_points(
            position_cones, self.triangulation_var_threshold
        )

        # Publish visualisation topics if needed
        if self.vis:
            self.publish_points(
                position_cones, frame_id, self.vis_namespace + "/position_cones", (1, 0.5, 0, 1)
            )
            self.publish_points(
                triangulation_centers, frame_id, self.vis_namespace + "/triangulation_centers", (1, 0, 0, 1)
            )
            self.publish_points(
                center_points,
                frame_id,
                self.vis_namespace + "/center_points",
                (0, 1, 0, 1),
                0.2
            )

        _, leaves = self.triangulation_paths.get_all_paths(
            triangulation_centers, center_points, cones[:, :-1]
        )

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

        costs = np.zeros((len(leaves), 5))
        paths = []

        # Iterate each leaf
        for i, leave in enumerate(leaves):

            # Find the path connecting this leaf to the root and reverse it
            path = []
            parent = leave
            while parent.parent is not None:
                path.append(parent)
                parent = parent.parent
            path.reverse()

            # Calculate the path cost and normalise it
            (
                color_cost,
                angle_cost,
                width_cost,
                spacing_cost,
                length_cost,
            ) = self.triangulation_paths.get_cost_branch(path, cones)
            costs[i] = [
                5 * color_cost,
                angle_cost,
                width_cost,
                spacing_cost,
                10 * length_cost,
            ]
            paths.append(path)

        np.set_printoptions(suppress=True)

        total_cost = np.sum(costs, axis=1)

        # Get the least-cost path
        index = np.argmin(total_cost)
        paths = np.array(paths)

        return paths[index]

    def publish_points(self, points: np.ndarray, header: Header, namespace: str, color: Tuple, scale : float = 0.1) -> None:
        """Publishes points to the already set vis_points topic

        Args:
            points: Nx2 array of point locations (x and y)
            header: the message header of the original observations
            namespace: the namespace to publish the markers to
            color: a tuple of RGBA values (between 0 and 1)
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
