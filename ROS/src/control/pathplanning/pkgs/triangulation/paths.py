import numpy as np
from pathplanning_dc.node import Node
import triangulation.utils as utils
from typing import Tuple
import math


class TriangulationPaths:
    def __init__(
        self,
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
    ) -> None:
        """Initialize triangulator

        Args:
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
        """
        self.max_iter = max_iter
        self.max_angle_change = max_angle_change
        self.max_path_distance = max_path_distance
        self.safety_dist = safety_dist
        self.safety_dist_squared = safety_dist**2

        self.continuous_dist = continuous_dist
        self.stage1_rectangle_width = stage1_rectangle_width
        self.stage1_bad_points_threshold = stage1_bad_points_threshold
        self.stage1_center_points_threshold = stage1_center_points_threshold
        self.stage2_rectangle_width = stage2_rectangle_width
        self.stage2_bad_points_threshold = stage2_bad_points_threshold
        self.stage2_center_points_threshold = stage2_center_points_threshold
        self.max_depth = max_depth

    def get_all_paths(
        self,
        bad_points: np.ndarray,
        center_points: np.ndarray,
        cones: np.ndarray,
        range_front: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get/generate all possible paths

        Args:
            triangulation_centers: center points of the edges of the triangles
            center_points: center points of the edges of the triangles that were duplicated,
                probably forming the center line
            cones: cones
            range_front: The lookahead distance for sorting points

        Returns:
        tuple of root node and leaves of paths
        """
        root_node = Node(0, 0, 0, None, [], 0, 0)
        queue = [(root_node, [(0, 0)], 0)]
        leaves = []

        iterations = 0
        while queue:
            iterations += 1

            if iterations >= self.max_iter:
                break

            parent, path, depth = queue.pop()

            # First stage is adding as much continuous nodes as possible to reduce search space by alot
            child_found = True
            while child_found:

                next_nodes = utils.sort_closest_to(
                    center_points, (parent.x, parent.y), self.continuous_dist
                )
                child_found = False
                for next_pos in next_nodes:

                    if utils.check_if_feasible_child(
                        parent,
                        path,
                        next_pos,
                        bad_points,
                        center_points,
                        cones,
                        self.max_angle_change,
                        self.safety_dist_squared,
                        self.stage1_rectangle_width,
                        self.stage1_bad_points_threshold,
                        self.stage1_center_points_threshold,
                    ):

                        distance_node = (parent.x - next_pos[0]) ** 2 + (
                            parent.y - next_pos[1]
                        ) ** 2
                        angle_node = np.arctan2(
                            next_pos[1] - parent.y, next_pos[0] - parent.x
                        )
                        angle_change = angle_node - parent.angle

                        # All good! Add this one
                        node = Node(
                            next_pos[0],
                            next_pos[1],
                            distance_node,
                            parent,
                            [],
                            angle_node,
                            angle_change,
                        )

                        # Move node
                        path += [(node.x, node.y)]
                        parent.children.append(node)
                        parent = parent.children[0]

                        child_found = True

                        # A single child is enough!
                        break

            # Now we are at the second stage. We basically look for a node in the distance (to bridge a gap) and add it to the stack to explore further
            # We also limit the depth of this stage
            if depth >= self.max_depth:
                leaves.append(parent)
                continue

            next_nodes = utils.sort_closest_to(
                center_points, (parent.x, parent.y), range_front
            )
            child_found = False
            for next_pos in next_nodes:

                if utils.check_if_feasible_child(
                    parent,
                    path,
                    next_pos,
                    bad_points,
                    center_points,
                    cones,
                    self.max_angle_change,
                    self.safety_dist_squared,
                    self.stage2_rectangle_width,
                    self.stage2_bad_points_threshold,
                    self.stage2_center_points_threshold,
                ):

                    distance_node = (parent.x - next_pos[0]) ** 2 + (
                        parent.y - next_pos[1]
                    ) ** 2
                    angle_node = np.arctan2(
                        next_pos[1] - parent.y, next_pos[0] - parent.x
                    )
                    angle_change = angle_node - parent.angle

                    # All good! Add this one
                    node = Node(
                        next_pos[0],
                        next_pos[1],
                        distance_node,
                        parent,
                        [],
                        angle_node,
                        angle_change,
                    )

                    # Move node
                    new_path = path + [(node.x, node.y)]
                    parent.children.append(node)

                    queue.append((node, new_path, depth + 1))

                    child_found = True

            # If no (valid) child can be found it means that we have found a leaf
            if not child_found:
                leaves.append(parent)

        return None, leaves

    def get_cost_branch(
        self,
        branch: np.ndarray,
        cones: np.ndarray,
        range_front: float,
    ) -> Tuple[float, float]:
        """Get cost of branch.

        Args:
            branch: branch to calculate cost for
            cones: cones
            range_front: The lookahead distance for sorting points

        Returns:
        tuple of angle_cost, color_cost, width_cost, spacing_cost, length_cost
        """
        # angle should not change much over one path
        angle_changes = np.array([abs(point.angle_change) for point in branch])
        angle_cost = np.var(angle_changes)

        # longer paths usually work better as they make use of more center points
        # having many segments along a path is usually a good path
        node_distances = np.array([point.distance for point in branch])
        distance = np.sum(node_distances)
        length_cost = 1 / distance + 1 / len(node_distances) * 10

        # get array of blue cones and array of yellow cones sorted by distance
        sorted_cones = utils.sort_closest_to(cones, (0, 0), range_front)
        blue_sorted = sorted_cones[sorted_cones[:, 2] == 0]
        yellow_sorted = sorted_cones[sorted_cones[:, 2] == 1]

        # Iterate over the path, get all cones in a certain distance to the line, and apply a penalty every time a cone is on the wrong side
        for point in branch:
            line_distances_squared = np.cos(point.angle) * (
                cones[:, 1] - point.y
            ) - np.sin(point.angle) * (cones[:, 0] - point.x)
            distances_squared = utils.distance_squared(
                point.x, point.y, cones[:, 0], cones[:, 1]
            )
            close_cones = cones[
                np.logical_and(
                    line_distances_squared < 3**2, distances_squared < 3**2
                )
            ]
            close_classes = close_cones[:, -1]

            diff_angles = (
                np.arctan2(close_cones[:, 1] - point.y, close_cones[:, 0] - point.x)
                - point.angle
            )

            # Left (positive angle) should be class 0, right (negative angle) should be class 1
            # Ignore class >= 2
            penalty_amount = np.sum(close_classes[diff_angles > 0.0] == 1) + np.sum(
                close_classes[diff_angles < 0.0] == 0
            )

            length_cost += penalty_amount * 10
            angle_cost += penalty_amount * 10

        # get center between closest blue cone and closest yellow cone
        center_point = (
            (blue_sorted[0][0] + yellow_sorted[0][0]) / 2,
            (blue_sorted[0][1] + yellow_sorted[0][1]) / 2,
        )
        for point in branch:
            if np.isclose(center_point[0], point.x) and np.isclose(
                center_point[1], point.y
            ):
                # if any point of the path is close to this center point, reduce cost
                length_cost /= 100
                angle_cost /= 100

        return angle_cost, length_cost
