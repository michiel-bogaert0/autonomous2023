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
                        atol=np.pi / 36,  # 5 degree sensitivity
                    )
                    if np.count_nonzero(within_range) > 0:
                        continue

                # Check that the angle change is within bounds
                if abs_angle_change > self.max_angle_change:
                    continue

                distance_node = (parent.x - next_pos[0]) ** 2 + (parent.y - next_pos[1]) ** 2
                # Check the distance between subsequent nodes in the planned path
                if distance_node > self.max_path_distance ** 2:
                    continue

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

        node_distances = np.array([point.distance for point in branch])
        distance = np.sum(node_distances)
        # length_cost = ((self.plan_dist - distance) / self.plan_dist) ** 2
        length_cost = 1 / distance + np.var(node_distances) / 20

        global_left_cones = []
        global_right_cones = []
        # Loop through branch and estimate cost
        for point in branch:
            neg_theta = -point.angle

            # Check cone colours left and right
            rot = np.array(
                [
                    [math.cos(neg_theta), -math.sin(neg_theta), 0],
                    [math.sin(neg_theta), math.cos(neg_theta), 0],
                    [0, 0, 1],
                ]
            )
            trans = np.array([[1, 0, -point.x], [0, 1, -point.y], [0, 0, 1]])

            # Get the homogeneous coordinates of the cones
            copied_cones = np.copy(cones)
            copied_cones[:, 2] = 1
            relative_cones = np.transpose(rot @ trans @ copied_cones.T)

            relative_cones[:, -1] = cones[:, -1]

            local_left_cones = relative_cones[relative_cones[:, 1] > 0]
            local_right_cones = relative_cones[relative_cones[:, 1] < 0]

            if len(local_left_cones) == 0 or len(local_right_cones) == 0:
                # No cones on one side, so probably outside of track
                # Possible problem corner
                return np.inf, np.inf, np.inf, np.inf, np.inf

            # Get the closest left and right cone (in global space) at this point in the branch
            # and add them to the global cone lists
            distances_squared_left = utils.distance_squared(
                0, 0, local_left_cones[:, 0], local_left_cones[:, 1]
            )
            closest_left = cones[relative_cones[:, 1] > 0][
                np.argmin(distances_squared_left)
            ]

            distances_squared_right = utils.distance_squared(
                0, 0, local_right_cones[:, 0], local_right_cones[:, 1]
            )
            closest_right = cones[relative_cones[:, 1] < 0][
                np.argmin(distances_squared_right)
            ]

            # Only add cones that are not yet in the global list
            if len(global_left_cones) == 0 or not np.any(
                np.all(np.isclose(np.array(global_left_cones), closest_left), axis=1)
            ):
                global_left_cones.append(closest_left)
            if len(global_right_cones) == 0 or not np.any(
                np.all(np.isclose(np.array(global_right_cones), closest_right), axis=1)
            ):
                global_right_cones.append(closest_right)

        global_right_cones_array = np.array(global_right_cones)
        global_left_cones_array = np.array(global_left_cones)

        if len(global_right_cones) > 0 and len(global_left_cones) > 0:
            # Count the cones on each side of the car that have the wrong colour
            color_cost = (
                (
                    np.count_nonzero(global_right_cones_array[:, -1] == 0)
                    + np.count_nonzero(global_left_cones_array[:, -1] == 1)
                )
                / len(cones)
            ) ** 2
        else:
            color_cost = 50

        track_widths = (
            []
        )  # We remember the predicted track width because this should stay relatively constant
        cone_spacings = []  # Same as width but for cone spacing
        # For each left cone, find the closest right one and use that to calculate the track width
        # For each left cone, find the closest left one and use that to calculate the cone spacing
        for cone_pos_left in global_left_cones_array:
            cone_pos_stacked = np.tile(
                cone_pos_left[:-1], (global_right_cones_array.shape[0], 1)
            )
            distances_squared = utils.distance_squared(
                cone_pos_stacked[:, 0],
                cone_pos_stacked[:, 1],
                global_right_cones_array[:, 0],
                global_right_cones_array[:, 1],
            )
            closest_dist_opposite = np.sqrt(np.min(distances_squared))

            if (
                len(global_left_cones_array) == 1
            ):  # There is only one same axis cone, add a penalty
                closest_dist_same = 10
            else:
                cone_pos_stacked = np.tile(
                    cone_pos_left[:-1], (global_left_cones_array.shape[0], 1)
                )
                distances_squared = utils.distance_squared(
                    cone_pos_stacked[:, 0],
                    cone_pos_stacked[:, 1],
                    global_left_cones_array[:, 0],
                    global_left_cones_array[:, 1],
                )

                # Don't forget the cone itself is in this result
                closest_dist_same = np.sqrt(np.sort(distances_squared)[1])

            track_widths.append(closest_dist_opposite)
            cone_spacings.append(closest_dist_same)

        for cone_pos_right in global_right_cones_array:
            cone_pos_stacked = np.tile(
                cone_pos_right[:-1], (global_left_cones_array.shape[0], 1)
            )
            distances_squared = utils.distance_squared(
                cone_pos_stacked[:, 0],
                cone_pos_stacked[:, 1],
                global_left_cones_array[:, 0],
                global_left_cones_array[:, 1],
            )
            closest_dist_opposite = np.sqrt(np.min(distances_squared))

            if (
                len(global_right_cones_array) == 1
            ):  # There is only one same axis cone, add a penalty
                closest_dist_same = 10
            else:
                cone_pos_stacked = np.tile(
                    cone_pos_right[:-1], (global_right_cones_array.shape[0], 1)
                )
                distances_squared = utils.distance_squared(
                    cone_pos_stacked[:, 0],
                    cone_pos_stacked[:, 1],
                    global_right_cones_array[:, 0],
                    global_right_cones_array[:, 1],
                )
                # Don't forget the cone itself is in this result
                closest_dist_same = np.sqrt(np.sort(distances_squared)[1])

            track_widths.append(closest_dist_opposite)
            cone_spacings.append(closest_dist_same)

        width_cost = np.var(track_widths) ** 2
        spacing_cost = np.var(cone_spacings) ** 2

        return angle_cost, color_cost, width_cost, spacing_cost, length_cost
