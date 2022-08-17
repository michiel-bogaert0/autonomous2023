import random

from dataclasses import dataclass
from typing import List, Tuple

import math
import numpy as np


@dataclass()
class RrtNode:
    """Defines a node in RRT tree"""

    index: int
    parent: "RrtNode"
    children: List["RrtNode"]

    def del_child(self, child: "RrtNode"):
        """Delete child from self.

        Args:
            child: Child node to delete.
        """
        self.children.remove(child)


class Rrt:
    """The RRT algorithm."""

    # Indexes of array points
    POINT_LEN = 8
    IDX_X = 0
    IDX_Y = 1
    IDX_ANGLE = 2
    IDX_MID_COST = 3
    IDX_DIST2ROOT = 4
    IDX_PAD2ROOT = 5
    IDX_HOPS2ROOT = 6
    IDX_LEAF = 7

    def __init__(
        self,
        max_dist: float = 0.5,
        plan_dist: float = 12.0,
        max_iter: int = 750,
        max_angle_change: float = 0.8,
        safety_dist: float = 0.3,
        track_width: float = 3,
        max_track_width: float = 4,
        search_rad: float = None,
        iter_threshold: int = 560,
        angle_inc: float = 0.2,
        angle_fac: float = 1.5,
    ):
        """Initialize RRT algoritme with parameters.

        Args:
            max_dist: The maximum distance between new node and closest other node.
            plan_dist: The maximum distance cones would be detected.
            max_iter: Maximum iteratioins RRT will run (not equal to amount of nodes)
            max_angle_change: Maximum angle change allowed between nodes.
            safety_dist: Extra distance from middle of car to keep from cones and other
            objects. Should be at least half the width of the car.
            track_width: Minimum or average width of the track.
            Used to estimate middle one side of cones is missing.
            max_track_width: Maximum track width.
            Used to detect if RRT node is possibly out of the track.
            search_rad: Radius in which neighbors will be iterate to optimize
            RRT node (part of RRT*).
            iter_threshold: Iteration threshold which triggers parameter update. (3/4 of max_iter seems to be ok)
            angle_inc: Percentage to increase maximum angle when parameter update is triggered.
            angle_fac: Factor in to increase maximum angle to create more chance for edges.
        """
        self.root = RrtNode(0, None, [])
        self.points = np.zeros((1, Rrt.POINT_LEN))
        self.nodes = [self.root]
        self.cones = None

        # Normal values
        self.max_dist_DEFAULT = max_dist
        self.plan_dist_DEFAULT = plan_dist
        self.max_iter_DEFAULT = max_iter
        self.max_angle_change_DEFAULT = max_angle_change
        self.max_cos_change_DEFAULT = math.cos(max_angle_change)
        self.safety_dist_DEFAULT = safety_dist
        self.safety_dist_squared_DEFAULT = safety_dist**2
        self.track_width_DEFAULT = track_width  # Minimum/average width of track
        self.max_track_width_squared_DEFAULT = (
            max_track_width**2
        )  # Maximum width of track
        self.iter_threshold_DEFAULT = iter_threshold
        self.angle_inc_DEFAULT = angle_inc
        self.angle_fac_DEFAULT = angle_fac
        # Set defaults
        self.reset_params()

        if search_rad is not None:
            self.search_rad = search_rad
        else:
            self.search_rad = 2 * max_dist

        # For later use to accumulate times a node is rejected
        self.safety_dist_faults = 0
        self.angle_faults = 0
        self.mid_faults = 0
        self.cut_faults = 0

        self.ewma_angle = 0.0
        self.ewma_alpha = 1 / 2

    def clear_faults(self):
        """Reset fault counters"""
        self.safety_dist_faults = 0
        self.angle_faults = 0
        self.mid_faults = 0
        self.cut_faults = 0

    def reset_params(self):
        """Reset parameters to default value."""
        self.max_dist = self.max_dist_DEFAULT
        self.plan_dist = self.plan_dist_DEFAULT
        self.max_iter = self.max_iter_DEFAULT
        self.max_angle_change = self.max_angle_change_DEFAULT
        self.max_cos_change = self.max_cos_change_DEFAULT
        self.safety_dist = self.safety_dist_DEFAULT
        self.safety_dist_squared = self.safety_dist_squared_DEFAULT
        self.track_width = self.track_width_DEFAULT
        self.max_track_width_squared = self.max_track_width_squared_DEFAULT
        self.iter_threshold = self.iter_threshold_DEFAULT
        self.angle_inc = self.angle_inc_DEFAULT
        self.angle_fac = self.angle_fac_DEFAULT

    def renew(self, cones: np.ndarray, root: Tuple[float, float] = None):
        """Renew algorithm with new cones.

        Args:
            cones: Cones to find path between
            root: Specify for non (0,0) root location (Default: None)
        """
        self.cones = cones

        self.clear_faults()

        self.reset_params()

        if root is not None:
            self.root = RrtNode(0, None, [])
            self.points = np.zeros((1, Rrt.POINT_LEN))
            self.points[self.root.index, [Rrt.IDX_X, Rrt.IDX_Y]] = np.array(
                [root[0], root[1]]
            )
            self.nodes = [self.root]

    def add_node(self, x: float, y: float):
        """
        Find nearest node
        Args:
            x: x-coordinate of new node
            y: y-coordinate of new node
        Returns:
            Status code (0: success | 1: within safety ditance | 2: out of boundary | 3: failed angle |
            4: too far from cone | 5: cuts off safety distance)
        """
        # Find closest node
        diff = (self.points[:, Rrt.IDX_X] - x) ** 2 + (
            self.points[:, Rrt.IDX_Y] - y
        ) ** 2
        idx_nearest = np.argmin(diff)
        nearest_node = self.nodes[idx_nearest]
        nearest_point = self.points[idx_nearest]
        # Rescale x and y to be within max_dist of the node
        new_x, new_y = self.rescale(nearest_point, x, y, self.max_dist)

        # Checks...
        cos_angle = self.calc_cos_angle(new_x, new_y, nearest_node)
        if self.check_angle(cos_angle=cos_angle):
            self.angle_faults += 1
            return 3

        if self.check_safety_distance(new_x, new_y):
            self.safety_dist_faults += 1
            return 1

        # Check neighbors and optimal path to root
        neighbor_idxs, dists = self.find_neighbor_idxs(new_x, new_y)

        min_cost = nearest_point[Rrt.IDX_PAD2ROOT] + dists[idx_nearest]
        idx_best = idx_nearest

        # DELETED: find better point

        # Get the corresponding RrtNode object
        best_node = nearest_node

        # Check if new edge would cut in safety distance of the cones
        if self.check_safety_distance_cutting(new_x, new_y, best_node):
            self.cut_faults += 1
            return 5

        # Calculate difference between closest yellow cone and blue cone
        mid_cost, min_b, min_y = self.calculate_cost(new_x, new_y, best_node)
        if min_b > self.max_track_width_squared or min_y > self.max_track_width_squared:
            self.mid_faults += 1
            return 4

        # Connect new node
        idx_new_node, new_node = self.connect_node(
            new_x, new_y, best_node, min_cost, cos_angle, mid_cost
        )

        # Reconnect neighbors for optimal path
        for i in neighbor_idxs:
            if i == idx_best or i == self.root.index:
                continue
            if (
                dists[i] + min_cost < self.points[i, Rrt.IDX_PAD2ROOT]
                and not self.check_angle(
                    self.points[i, Rrt.IDX_X], self.points[i, Rrt.IDX_Y], new_node
                )
                and not self.check_safety_distance_cutting(
                    self.points[i, Rrt.IDX_X], self.points[i, Rrt.IDX_Y], new_node
                )
            ):
                # Reconnect neighbor node i to new node
                neighbor = self.nodes[i]
                parent = neighbor.parent
                parent.del_child(neighbor)
                if not parent.children:
                    # Set current node as leaf if no children left
                    self.points[parent.index, Rrt.IDX_LEAF] = 1
                neighbor.parent = new_node
                new_node.children.append(neighbor)
                self.points[idx_new_node, Rrt.IDX_LEAF] = 0

        # Node added successfully
        return 0

    def connect_node(
        self,
        new_x: float,
        new_y: float,
        parent: "RrtNode",
        pad2root: float,
        cos_angle: float,
        mid_cost: float,
    ) -> Tuple[int, "RrtNode"]:
        """
        Connects new node to tree
        Args:
            new_x: x-coordinate of node to add
            new_y: y-coordinate of node to add
            pad2root: distance of path to root of node to add
            cos_angle: angle between node, parent and grandparent
            parent: parent node to which it will be connected
            mid_cost: cost of difference in the closest yellow and blue cone

        Returns:
        index of new node, new node
        """
        # Determine index of new node
        idx_new_node = len(self.points)
        # idx_new_node = len(self.nodes)
        # Make new node
        new_node = RrtNode(idx_new_node, parent, [])

        # Construct new point

        # Calculate cumulative angle
        angle_cost = self.points[parent.index, Rrt.IDX_ANGLE] + (1 - cos_angle)
        # Calculate difference between distance to root and plan distance
        root_point = self.points[self.root.index]
        dist2root = (root_point[Rrt.IDX_X] - new_x) ** 2 + (
            root_point[Rrt.IDX_Y] - new_y
        ) ** 2
        dist_cost = (self.plan_dist**2 - dist2root) ** 2

        newpoint = np.zeros(Rrt.POINT_LEN)
        newpoint[
            [
                Rrt.IDX_X,
                Rrt.IDX_Y,
                Rrt.IDX_ANGLE,
                Rrt.IDX_MID_COST,
                Rrt.IDX_DIST2ROOT,
                Rrt.IDX_PAD2ROOT,
                Rrt.IDX_HOPS2ROOT,
                Rrt.IDX_LEAF,
            ]
        ] = np.array(
            [
                new_x,
                new_y,
                angle_cost,
                mid_cost,
                dist_cost,
                pad2root,
                self.points[parent.index, Rrt.IDX_HOPS2ROOT] + 1,
                1,
            ]
        )

        # Add new node/point to points
        self.points = np.append(
            self.points, newpoint.reshape((1, Rrt.POINT_LEN)), axis=0
        )
        # Add new node to nodes list
        self.nodes.append(new_node)
        # Add new node to the children of its parent
        if not parent.children:
            self.points[parent.index, Rrt.IDX_LEAF] = 0
        parent.children.append(new_node)
        return idx_new_node, new_node

    def find_neighbor_idxs(self, x: float, y: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Find nodes within search_rad of coordinates
        Args:
            x: x-coordinate
            y: y-coordinate

        Returns:
        Indexes of neighbor nodes
        """
        idxs = np.arange(len(self.points), dtype=int)

        dists = (self.points[:, Rrt.IDX_X] - x) ** 2 + (
            self.points[:, Rrt.IDX_Y] - y
        ) ** 2
        kwad_dist = self.search_rad**2
        return idxs[dists < kwad_dist], dists

    def calculate_cost(self, x: float, y: float, parent: "RrtNode", *args, **kwargs):
        """
        Calculates cost of given coordinates based on difference in distances
        between neerest yellow and nearest blue cone.
        Args:
            x: x-coordinate
            y: y-coordinate
            parent: parent node
        Returns:
        Cost of a new node with given coordinates
        """
        # Get cones per color
        blue = self.cones[self.cones[:, 2] == 0]
        yellow = self.cones[self.cones[:, 2] == 1]
        if blue.shape[0] != 0:
            diff_b = (blue[:, Rrt.IDX_X] - x) ** 2 + (blue[:, Rrt.IDX_Y] - y) ** 2
        else:
            # No blue cones in sight
            diff_b = np.array([self.track_width**2 / 4])
        if yellow.shape[0] != 0:
            diff_y = (yellow[:, Rrt.IDX_X] - x) ** 2 + (yellow[:, Rrt.IDX_Y] - y) ** 2
        else:
            # No yellow cones in sight
            diff_y = np.array([self.track_width**2 / 4])

        min_b = np.min(diff_b)
        min_y = np.min(diff_y)
        return (
            self.points[parent.index, Rrt.IDX_MID_COST] + (min_b - min_y) ** 2,
            min_b,
            min_y,
        )

    def rescale(
        self, point: np.ndarray, x: float, y: float, max_distance: float
    ) -> Tuple[float, float]:
        """
        Rescale coordinates x and y to max_distance
        Args:
            point: point to rescale to
            x: x-coordinates to rescale
            y: y-coordinate to rescale
            max_distance: maximum distance from node n

        Returns:
            New rescaled coordinates
        """

        # Calculate vector from node to new coordinates
        vec_x = x - point[Rrt.IDX_X]
        vec_y = y - point[Rrt.IDX_Y]
        # Calculate length of vector or distance between node and new coordinate
        dist = math.sqrt(vec_x**2 + vec_y**2)
        if dist <= max_distance and False:
            return x, y
        else:
            return (
                point[Rrt.IDX_X] + vec_x * max_distance / dist,
                point[Rrt.IDX_Y] + vec_y * max_distance / dist,
            )

    def get_cost_branch(
        self, branch: np.ndarray, cones: np.ndarray
    ) -> Tuple[float, float, float]:
        """Get cost of specied branch based on the cones.

        Args:
            branch: Branch for which to calculate cost.
            cones: Cones based on which to calculate cost.

        Returns:
        tuple of color cost, width cost and spacing cost
        """
        # Maybe average angle change
        max_angle_changes = np.var(branch[:, self.IDX_ANGLE])
        max_angle_var = (self.max_cos_change / 2) ** 2
        angle_cost = (max_angle_changes / max_angle_var) ** 2

        # Choose one of 2:
        # Absolute distance
        distance = branch[-1, self.IDX_DIST2ROOT]
        # Path length from root to leaf
        distance = branch[-1, self.IDX_PAD2ROOT]

        length_cost = ((self.plan_dist - distance) / self.plan_dist) ** 2

        global_left_cones = []
        global_right_cones = []
        angles = np.zeros(branch.shape[-2])
        angles[1:] = np.arctan2(
            branch[1:, self.IDX_Y] - branch[:-1, self.IDX_Y],
            branch[1:, self.IDX_X] - branch[:-1, self.IDX_X],
        )
        # Loop through branch and estimate cost
        for idx, point in enumerate(branch):
            # Check average angle change
            neg_theta = -angles[idx]

            # Check cone colours left and right
            rot = np.array(
                [
                    [math.cos(neg_theta), -math.sin(neg_theta), 0],
                    [math.sin(neg_theta), math.cos(neg_theta), 0],
                    [0, 0, 1],
                ]
            )
            trans = np.array(
                [[1, 0, -point[self.IDX_X]], [0, 1, -point[self.IDX_Y]], [0, 0, 1]]
            )

            # Get the homogeneous coordinates of the cones
            copied_cones = np.copy(cones)
            copied_cones[:, 2] = 1
            relative_cones = np.transpose(rot @ trans @ copied_cones.T)

            relative_cones[:, -1] = cones[:, -1]

            local_left_cones = relative_cones[relative_cones[:, 1] > 0]
            local_right_cones = relative_cones[relative_cones[:, 1] < 0]

            if len(local_left_cones) == 0 or len(local_right_cones) == 0:
                print(f"No cones on one side, so probably outside of track")
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
                np.count_nonzero(np.array(global_right_cones_array)[:, -1] == 0)
                + np.count_nonzero(np.array(global_left_cones_array)[:, -1] == 1)
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

        # print(f'angle: {angle_cost:>3.5f} - colùsfjqslfour: {color_cost:3.5f} - width: {width_cost:3.5f} '
        #      f'- spacing: {spacing_cost:3.5f}- length: {length_cost:3.5f}')

        return color_cost, width_cost, spacing_cost

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

    def calculate_path(self) -> Tuple[float, np.ndarray]:
        """
        Calculate most optimal path and its cost
        Returns:
            cost, numpy array of most optimal path
        """
        path = np.array([], dtype=object)
        branch = np.empty((1, self.POINT_LEN))
        tot_cost = 0

        rows, last_idx = self.points.shape
        leaves = np.c_[self.points, np.arange(rows)]
        leaves = leaves[leaves[:, Rrt.IDX_LEAF] == 1]
        # Calculate costs...
        # Difference between sensor distance and actual distance of the leaf
        leaf_cost = (
            (leaves[:, Rrt.IDX_DIST2ROOT] - self.plan_dist) / self.plan_dist
        ) ** 2
        # Average angle change
        angle_cost = (
            leaves[:, Rrt.IDX_ANGLE]
            / leaves[:, Rrt.IDX_HOPS2ROOT]
            / self.max_angle_change
        ) ** 2
        # Middle cost
        mid_cost = leaves[:, Rrt.IDX_MID_COST]

        tot_cost = 10 * leaf_cost + angle_cost + 70 * mid_cost

        if len(tot_cost) == 0:
            parent = Node(0, 0, 0, None, [], 0, 0)
            return 0, np.array(
                [parent, Node(self.max_dist / 2, 0, 0, parent, [], 0, 0)]
            )
        # Choose lowest
        idx_best = np.argmin(tot_cost)
        idx_best = int(leaves[idx_best, last_idx])

        node = self.nodes[idx_best]
        while node is not None:
            idx = node.index

            # which distance?
            dist = 0
            # Angle in cosinus? -> convert if possible
            angle = self.points[idx, Rrt.IDX_ANGLE]
            # Angle change still to calculate
            angle_change = 0

            path = np.append(
                [
                    Node(
                        self.points[idx, Rrt.IDX_X],
                        self.points[idx, Rrt.IDX_Y],
                        dist,
                        node,
                        [],
                        angle,
                        angle_change,
                    )
                ],
                path,
                axis=0,
            )
            branch = np.append(branch, [self.points[idx]], axis=0)
            node = node.parent

        # Test get_cost_branch()
        # print(self.get_cost_branch(branch, self.cones))

        return tot_cost, path

    def calc_cos_angle(self, x: float, y: float, node: "RrtNode") -> float:
        """

        Args:
            x: x-coordinate of the node we want to add
            y: y-coordinate of the node we want to add
            node: node we want to add the new node to

        Returns:
            Angle, returns None if too big
        """
        node_point = self.points[node.index]

        # if node is root, check differently
        if node.index == 0:
            parent_point = np.array([-1, 0])
        else:
            parent_point = self.points[node.parent.index]
        vector_a = np.array(
            [
                node_point[Rrt.IDX_X] - parent_point[Rrt.IDX_X],
                node_point[Rrt.IDX_Y] - parent_point[Rrt.IDX_Y],
            ]
        )
        vector_b = np.array([x - node_point[Rrt.IDX_X], y - node_point[Rrt.IDX_Y]])
        norm = np.linalg.norm((vector_a, vector_b), axis=1)
        cos_angle = vector_a @ vector_b / (norm[0] * norm[1])
        return cos_angle

    def check_angle(
        self,
        x: float = None,
        y: float = None,
        parent: "RrtNode" = None,
        cos_angle: float = None,
    ) -> bool:
        """
        Checks if cosinus of angle is does not exceed maximum angle
        Args:
            x: x-coordinate of new point
            y: y-coordinate of new point
            parent: parent node
            cos_angle: cosinus of angle (in this case no calculations needed)

        Returns:
            True -> Angle too big | False -> Angle ok
        """
        if cos_angle is None:
            return self.calc_cos_angle(x, y, parent) < self.max_cos_change
        else:
            return cos_angle < self.max_cos_change

    def check_safety_distance(self, x: float, y: float) -> bool:
        """Check if point is within safety distance of object.

        Args:
            x: x-coordinate we want to check
            y: y-coordinate we want to check

        Returns:
            True or False: True if the coordinates are in the safety_distance, false if not
        """
        # Checks if point within safety distance of cone
        return np.any(
            ((self.cones[:, 0] - x) ** 2 + (self.cones[:, 1] - y) ** 2)
            <= self.safety_dist_squared
        )

    def check_safety_distance_cutting(
        self, x: float, y: float, parent: "RrtNode"
    ) -> bool:
        """
        Checks if new point would cut in safety distance of a cone
        Args:
            x: x-coordinate new point
            y: y-coordinate new point
            parent: parent node (where it would be connected to)

        Returns:
            True if it would cut through safety distance around cone, false if not
        """

        p1 = self.points[parent.index, [Rrt.IDX_X, Rrt.IDX_Y]]
        p2 = np.array([x, y])

        t = np.sum((p1 - p2) * (p1 - self.cones[:, [0, 1]]), axis=1)

        t /= np.sum(np.power(p2 - p1, 2))

        # Check if t in [0,1]
        t_filter = np.bitwise_and(t < 1, t > 0)

        p = p1 + (p2 - p1) * t[t_filter, None]

        if len(p) == 0:
            return False
        return np.any(
            np.sum(np.power(p - self.cones[t_filter][:, [0, 1]], 2), axis=1)
            < self.safety_dist_squared
        )

    def update_params(self):
        """Update parameters if required."""
        sum = (
            self.safety_dist_faults
            + self.angle_faults
            + self.mid_faults
            + self.cut_faults
        )
        # Possible improvements: * Increase iteration count when faults exceed threshold,
        # but limit this to avoid continously inceasing iterations
        #                        * Keep track of compleet amount of faults (don't reset)
        if sum > self.iter_threshold:
            # print("{:<15}|{:<15}|{:<15}|{:<15}".format("Safety_dist", "Angle", "Mid", "Safety_cut"))
            # print(f"{self.safety_dist_faults:<15}|{self.angle_faults:<15}|{self.mid_faults:<15}|{self.cut_faults:<15}")

            self.clear_faults()

            self.max_angle_change *= 1 + self.angle_inc
            self.max_angle_change = min(self.max_angle_change, math.pi / 2)
            self.max_cos_change = math.cos(self.max_angle_change)
            # print(f"New angle change: {self.max_angle_change} - New cos angle: {self.max_cos_change}")

    def rdm_new_node(self) -> Tuple[float, float]:
        """
        Generate random point within distance and angle in front of car
        Returns:
            Tuple of x- and y-coordinate
        """
        d = self.plan_dist
        r = random.random() * d
        theta = 2 * (random.random() - 0.5) * self.max_angle_change * self.angle_fac
        return r * math.cos(theta), r * math.sin(theta)

    def get_path(self, front_cones):
        """
        Run complete algorithm on front_cones
        Args:
            front_cones: cones given by perception

        Returns:
            Calculated path to follow
        """
        return self.run(front_cones)[1]

    def run(self, front_cones):
        """
        Runs complete algorithm
        Args:
            front_cones: cones given by perception

        Returns:
            Tuple of cost and calculated path
        """
        self.renew(front_cones, (0, 0))

        # Iterations should be varied together with threshold of update_params
        for i in range(self.max_iter):
            done = False
            new_x, new_y = self.rdm_new_node()
            done = self.add_node(new_x, new_y) == 0
            if not done:
                self.update_params()

        cost, path = self.calculate_path()

        # EWMA
        # NOTE: Resizes path[1] to length 1
        path[1] = self.path_EWMA(path[1])

        return cost, path

    def path_EWMA(self, node: "Node"):
        """
        Calculates Exponential Weighted Moving Average of the angle for the first node
        Args:
            node: node to calculate angle from

        Returns:
            Adapted node (unnecessary since overwritten)
        """
        rt = np.arctan2(node.y, node.x)
        self.ewma_angle = self.ewma_alpha * rt + (1 - self.ewma_alpha) * self.ewma_angle
        # Calculate new node position with length 1
        # Overwrite node position
        node.x = np.cos(self.ewma_angle)
        node.y = np.sin(self.ewma_angle)
        return node
