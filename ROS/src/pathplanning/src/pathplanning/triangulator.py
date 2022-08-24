import math
import sys
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay

from pathplanning.dataclasses.node import Node


class Triangulator:
    def __init__(
        self,
        max_iter: int,
        plan_dist: float,
        max_angle_change: float,
        safety_dist: float,
    ) -> None:
        """Initialize triangulator

        Args:
            max_iter: Amount of iterations
            plan_dist: Maximum distance to plan path
            max_angle_change: Maximum angle change in path
            safety_dist: Safety distance from objects
        """
        self.max_iter = max_iter
        self.plan_dist = plan_dist
        self.max_angle_change = max_angle_change
        self.safety_dist = safety_dist
        self.safety_dist_squared = safety_dist**2
        self.prev = None

    def get_path(self, cones: np.ndarray) -> np.ndarray:
        """Generate path based on the cones.

        Args:
            cones: Cones between which to generate path.

        Returns:
            path
        """
        position_cones = cones[:, :-1]

        if (len(position_cones) < 4):
            return None
            
        center_points, unique_center_points = self.get_center_points(position_cones)

        root_node, leaves = self.get_all_paths(
            center_points, unique_center_points, cones[:, :-1]
        )

        path = self.get_best_path(leaves, cones)

        return path

    def get_best_path(self, leaves: list, cones: np.ndarray) -> np.ndarray:
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

        for i, leave in enumerate(leaves):
            path = []
            parent = leave
            while parent.parent is not None:
                path.append(parent)
                parent = parent.parent
            path.reverse()

            (
                color_cost,
                angle_cost,
                width_cost,
                spacing_cost,
                length_cost,
            ) = self.get_cost_branch(path, cones)
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

        index = np.argmin(total_cost)

        return np.ndarray(paths[index])

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

        distance = np.sum([point.distance for point in branch])
        length_cost = ((self.plan_dist - distance) / self.plan_dist) ** 2

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
            distances_squared_left = self.distance_squared(
                0, 0, local_left_cones[:, 0], local_left_cones[:, 1]
            )
            closest_left = cones[relative_cones[:, 1] > 0][
                np.argmin(distances_squared_left)
            ]

            distances_squared_right = self.distance_squared(
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

        # Count the cones on each side of the car that have the wrong colour
        color_cost = (
            (
                np.count_nonzero(global_right_cones_array[:, -1] == 0)
                + np.count_nonzero(global_left_cones_array[:, -1] == 1)
            )
            / len(cones)
        ) ** 2

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
            distances_squared = self.distance_squared(
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
                distances_squared = self.distance_squared(
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
            distances_squared = self.distance_squared(
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
                distances_squared = self.distance_squared(
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

    def get_center_points(
        self, position_cones: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get center points of the edges of the triangles
        Args:
            position_cones: position of the cones

        Returns:
        tuple of center_points, unique_center_points
        """

        tri = Delaunay(position_cones)
        indices = tri.simplices
        triangles = position_cones[indices]

        self.prev = [position_cones, indices]

        # Waiting if frame is okay

        center_points = (triangles + np.roll(triangles, 1, axis=1)) / 2
        flattened_center_points = center_points.reshape((center_points.size // 2, 2))
        unique_center_points = np.unique(flattened_center_points, axis=0)
        return center_points, unique_center_points

    def get_all_paths(
        self,
        center_points: np.ndarray,
        unique_center_points: np.ndarray,
        cones: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get/generate all possible paths

        Args:
            center_points: center points of the edges of the triangles
            unique_center_points: unique center points of the edges of the triangles
            cones: cones

        Returns:
        tuple of root node and leaves of paths
        """
        root_node = Node(0, 0, 0, None, [], 0, 0)
        queue = []

        next_nodes = self.get_closest_center(unique_center_points, 3)
        for next_pos in next_nodes:
            angle_next = np.arctan2(next_pos[1], next_pos[0])
            distance_next = np.sqrt(
                self.distance_squared(0, 0, next_pos[0], next_pos[1])
            )
            angle_change_next = min(abs(angle_next), 2 * np.pi - abs(angle_next))
            next_node = Node(
                next_pos[0],
                next_pos[1],
                distance_next,
                root_node,
                [],
                angle_next,
                angle_change_next,
            )
            root_node.children.append(next_node)
            queue.append(next_node)

        leaves = []
        iteration = 0
        while queue and iteration < self.max_iter:
            iteration += 1
            parent = queue.pop(0)
            triangles = np.where(
                np.all(center_points == np.array([parent.x, parent.y]), axis=2)
            )[0]
            found_child = False
            for triangle in triangles:
                for i in range(3):
                    point = center_points[triangle][i]
                    if point[0] != parent.x or point[1] != parent.y:
                        angle_node = np.arctan2(
                            point[1] - parent.y, point[0] - parent.x
                        )
                        angle_change = angle_node - parent.angle
                        abs_angle_change = min(
                            abs(angle_change), 2 * np.pi - abs(angle_change)
                        )
                        distance_node = np.sqrt(
                            self.distance_squared(
                                parent.x, parent.y, point[0], point[1]
                            )
                        )

                        if (
                            abs_angle_change <= self.max_angle_change
                            and self.no_collision(parent, point, cones)
                        ):
                            node = Node(
                                point[0],
                                point[1],
                                distance_node,
                                parent,
                                [],
                                angle_node,
                                angle_change,
                            )
                            queue.append(node)
                            parent.children.append(node)
                            found_child = True
            if not found_child:
                leaves.append(parent)

        return root_node, leaves + queue

    def distance_squared(self, x1, y1, x2, y2):
        """Calculate squared distance between 2 points

        Args:
            x1: x-coordinate of first point
            y1: y-coordinate of first point
            x2: x-coordinate of second point
            y2: y-coordinate of second point

        Returns:
        the distance between the points
        """
        return np.power(x1 - x2, 2) + np.power(y1 - y2, 2)

    def no_collision(self, parent: Node, point: np.ndarray, cones: np.ndarray) -> bool:
        """Detect if no collision happened from between path from node and point and safety distance around cones.

        Args:
            parent: Node to start from
            point: Point to check collision path
            cones: cones which can't be collided with

        Returns:
        True if no collision happened else False
        """

        # New implementation
        x1 = parent.x
        y1 = parent.y
        x2 = point[0]
        y2 = point[1]
        xc = cones[:, 0]
        yc = cones[:, 1]

        t_numerator = (x1 - xc) * (x1 - x2) + (y1 - yc) * (y1 - y2)
        t_denominator = (x1 - x2) ** 2 + (y1 - y2) ** 2

        t = t_numerator / t_denominator

        t[t < 0] = 0
        t[t > 1] = 1

        xp = x1 + (x2 - x1) * t
        yp = y1 + (y2 - y1) * t

        dist_squared = self.distance_squared(xc, yc, xp, yp)

        return np.all(dist_squared >= self.safety_dist_squared)

    def get_closest_center(
        self, unique_center_points: np.ndarray, amount: int
    ) -> np.ndarray:
        """Get closest center points to root

        Args:
            unique_center_points: All unique center points
            amount: Amount of closest center points to extract

        Returns:
        array of closest center points with length "amount"
        """
        distances_squared = self.distance_squared(
            0, 0, unique_center_points[:, 0], unique_center_points[:, 1]
        )
        ind = np.argpartition(distances_squared, amount)
        return unique_center_points[ind[:amount]]
