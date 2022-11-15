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
    ) -> None:
        """Initialize triangulator

        Args:
            max_iter: Amount of iterations
            max_angle_change: Maximum angle change in path
            max_path_distance: Maximum distance between nodes in the planned path
            safety_dist: Safety distance from objects
        """
        self.max_iter = max_iter
        self.max_angle_change = max_angle_change
        self.max_path_distance = max_path_distance
        self.safety_dist = safety_dist
        self.safety_dist_squared = safety_dist**2

        self.angle_sensitivity = np.deg2rad(20)

    def get_all_paths(
        self,
        triangulation_centers: np.ndarray,
        center_points: np.ndarray,
        cones: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get/generate all possible paths

        Args:
            triangulation_centers: center points of the edges of the triangles
            center_points: center points of the edges of the triangles that were duplicated,
                probably forming the center line
            cones: cones

        Returns:
        tuple of root node and leaves of paths
        """
        root_node = Node(0, 0, 0, None, [], 0, 0)
        queue = [root_node]
        leaves = []
        iteration = 0
        while queue and iteration < self.max_iter:
            iteration += 1
            found_child = False

            # Get the next element from the queue
            parent = queue.pop(0)

            # Get the closest center points to this element that are within 6m
            next_nodes = utils.sort_closest_to(center_points, (parent.x, parent.y), 6)

            # A list of all the abs_angle_change to nodes seen thus far
            angles_added = None

            # Iterate over each next_node, calculate the metrics and add the node to the tree
            # we also perform some early pruning based on the angle between nodes
            for next_pos in next_nodes:

                # Make sure we're not looping from the parent to itself
                if np.isclose(next_pos[0], parent.x) and np.isclose(
                    next_pos[1], parent.y
                ):
                    continue

                angle_node = np.arctan2(next_pos[1] - parent.y, next_pos[0] - parent.x)
                angle_change = angle_node - parent.angle

                abs_angle_change = min(abs(angle_change), 2 * np.pi - abs(angle_change))

                # We want to actually physically branch out, so only add one branch for each direction
                # By starting with the closest points, we hope this is a good point for a given direction
                if angles_added is not None:
                    within_range = np.isclose(
                        angles_added,
                        np.repeat(abs_angle_change, len(angles_added)),
                        atol=self.angle_sensitivity,
                    )
                    if np.count_nonzero(within_range) > 0:
                        continue

                # Check that the angle change is within bounds
                if abs_angle_change > self.max_angle_change:
                    continue
                
                distance_node = (parent.x - next_pos[0]) ** 2 + (parent.y - next_pos[1]) ** 2
                
                # Check we're not colliding with a cone
                if utils.no_collision(
                    parent, next_pos, cones, self.safety_dist_squared
                ):
                    node = Node(
                        next_pos[0],
                        next_pos[1],
                        distance_node,
                        parent,
                        [],
                        angle_node,
                        angle_change,
                    )
                    
                    if angles_added is None:
                        angles_added = np.array([abs_angle_change])
                    else:
                        angles_added = np.append(angles_added, abs_angle_change)

                    queue.append(node)
                    parent.children.append(node)
                    found_child = True

            # Check whether this node is a child node
            if not found_child:
                leaves.append(parent)

        return root_node, leaves + queue

    def get_cost_branch(
        self, branch: np.ndarray, cones: np.ndarray
    ) -> Tuple[float, float, float, float, float]:
        """Get cost of branch.

        Args:
            branch: branch to calculate cost for
            cones: cones

        Returns:
        tuple of angle_cost, color_cost, width_cost, spacing_cost, length_cost
        """
        # Maybe average angle change
        var_angle_change = np.var([point.angle_change for point in branch])
        max_angle_var = (self.max_angle_change / 2) ** 2
        angle_cost = (var_angle_change / max_angle_var) ** 2

        length_cost = np.var([point.distance for point in branch])

        return angle_cost, length_cost
