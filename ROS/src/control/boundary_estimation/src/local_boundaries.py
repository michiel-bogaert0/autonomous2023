from math import copysign

import numpy as np
import rospy
from node import Node


class LocalBoundaries:
    def __init__(self):
        self.max_dist = rospy.get_param("~max_distance", 36)
        self.min_dist = rospy.get_param("~min_distance", 0.5)
        self.max_turn = rospy.get_param("~max_angle", 90) * np.pi / 180
        self.dist_factor = rospy.get_param("~dist_factor", 1)
        self.turn_factor = rospy.get_param("~turn_factor", 1)
        self.turn_change_factor = rospy.get_param("~turn_change_factor", 1)
        self.use_color = rospy.get_param("~use_color", False)
        self.bound_len = rospy.get_param("~local_bound_len", 6)
        self.strategy = rospy.get_param("~middle_line_strategy", "distance")

    def get_boundaries(self, cones):
        """
        Computes the left and right boundaries from a set of cones, using the closest cone nodes
        to iteratively expand each boundary until a specified length or boundary conditions are met.

        Parameters:
        cones (list): List of cones to build boundaries from.

        Returns:
        tuple: The left and right boundaries as lists of nodes, or (None, None) if boundaries cannot be formed.
        """
        # Initialize boundary-related variables
        self.cones = cones
        self.left_bound, self.right_bound = [], []
        self.left_forbidden, self.right_forbidden = [], []

        # Create and add a root node as the starting point for both boundaries
        root_node = Node(0, 0, 2, 0, None, [], 0, 0, 0, None)
        self.left_bound.append(root_node)
        self.right_bound.append(root_node)

        # Find the closest nodes on each side of the root node to start the boundaries
        closest_left_node = self.get_closest_node(root_node, "left")
        closest_right_node = self.get_closest_node(root_node, "right")
        self.left_bound.append(closest_left_node)
        self.right_bound.append(closest_right_node)

        # If no initial nodes are found on either side, return None for both boundaries
        if closest_left_node is None or closest_right_node is None:
            return None, None

        # Iteratively grow the boundaries by adding new nodes until reaching the specified length
        iteration_count = 0
        while iteration_count < self.bound_len and (
            len(self.left_bound) + len(self.right_bound) - 2
        ) < len(self.cones):
            left_first, right_first = (
                len(self.left_bound) == 2,
                len(self.right_bound) == 2,
            )

            # Calculate angles between the last nodes of each boundary to decide expansion direction
            crossline_angle_left = np.arctan2(
                self.right_bound[-1].y - self.left_bound[-1].y,
                self.right_bound[-1].x - self.left_bound[-1].x,
            )
            crossline_angle_right = np.arctan2(
                self.left_bound[-1].y - self.right_bound[-1].y,
                self.left_bound[-1].x - self.right_bound[-1].x,
            )
            cross_angle_left = abs(
                np.pi - abs(crossline_angle_left - self.left_bound[-1].angle)
            )
            cross_angle_right = abs(
                np.pi - abs(crossline_angle_right - self.right_bound[-1].angle)
            )

            # Add next node to the boundary with a smaller cross angle
            if cross_angle_left > cross_angle_right:
                next_left_node = self.get_next_node("left", left_first)
                if next_left_node is not None:
                    self.left_bound.append(next_left_node)

            # Add next node to the right boundary if necessary
            if cross_angle_right >= cross_angle_left or next_left_node is None:
                next_right_node = self.get_next_node("right", right_first)
                if next_right_node is not None:
                    self.right_bound.append(next_right_node)
                else:
                    break  # Exit loop if no further right node can be found

            iteration_count += 1

        # Return None for both boundaries if they cannot be expanded, otherwise return the boundaries
        if not self.left_bound or not self.right_bound:
            return None, None

        return self.left_bound, self.right_bound

    def get_closest_node(self, root_node, side):
        """
        Finds the closest cone node to the given root node on the specified side (left or right).

        Args:
            root_node (Node): The starting node from which to search for the closest cone.
            side (str): The side to search for the closest cone. Must be either "left" or "right".

        Returns:
            Node: The closest cone node on the specified side, or None if no cone is found.
        """
        closest_cone = None
        closest_dist = float("inf")

        for cone in self.cones:
            dist = self.distance(root_node, cone)
            angle = np.arctan2(cone[1] - root_node.y, cone[0] - root_node.x)

            if side == "left" and angle > 0 and angle < self.max_turn:
                if dist < closest_dist:
                    closest_dist = dist
                    closest_cone = cone

            elif side == "right" and angle < 0 and angle > -self.max_turn:
                if dist < closest_dist:
                    closest_dist = dist
                    closest_cone = cone
        if closest_cone is None:
            return None
        closest_node = Node(
            x=closest_cone[0],
            y=closest_cone[1],
            color=closest_cone[2],
            distance=closest_dist,
            parent=root_node,
            children=[],
            angle=0,
            turn=0,
            cost=0,
            id=closest_cone[3],
        )
        root_node.children.append(closest_node)
        return closest_node

    def get_next_node(self, side, first):
        """
        Determines the next node to add to the specified boundary (left or right)
        based on the closest nodes in terms of cost and line alignment.

        Parameters:
        side (str): 'left' or 'right' to specify which boundary to expand.
        first (bool): Indicator of the first node addition to the boundary.

        Returns:
        Node or None: The next best node for the specified boundary, or None if no suitable node is found.
        """
        # Set up variables based on the chosen side
        if side == "left":
            bound, other_bound = self.left_bound, self.right_bound
            forbidden, forbidden_other = self.left_forbidden, self.right_forbidden
        elif side == "right":
            bound, other_bound = self.right_bound, self.left_bound
            forbidden, forbidden_other = self.right_forbidden, self.left_forbidden

        # Collect nodes that aren't in line with the current boundary or forbidden nodes
        nodes = [
            self.create_node(bound[-1], cone, first)
            for cone in self.cones
            if not self.is_in_line(cone, bound) and not self.is_in_line(cone, forbidden)
        ]
        nodes = [node for node in nodes if node is not None]

        # Sort nodes by increasing cost
        best_nodes = sorted(nodes, key=lambda node: node.cost)

        # Iterate through best nodes to find one suitable for the boundary
        for nr, best_node in enumerate(best_nodes):
            if self.is_in_line(best_node, other_bound):
                for i, node in enumerate(other_bound):
                    if node.x == best_node.x and node.y == best_node.y:
                        # Remove children already in the current boundary
                        node.parent.children = [
                            child
                            for child in node.parent.children
                            if not self.is_in_line(child, bound)
                        ]

                        if nr + 1 < len(best_nodes) and len(node.parent.children) > 1:
                            # Check if the current best_node should replace other_bound node
                            if (
                                node.parent.children[1].cost < best_nodes[nr + 1].cost
                                and self.use_color
                            ) or (
                                (side == "left" and best_node.color == 0)
                                or (side == "right" and best_node.color == 1)
                                and not self.use_color
                            ):
                                # Update boundaries, removing forbidden nodes from other_bound
                                other_bound[:] = other_bound[:i]
                                other_bound[-1].children.pop(0)
                                other_bound.append(other_bound[-1].children[0])
                                forbidden_other.append(node)

                                # Exclude nodes that already belong to other_bound
                                children = [
                                    child
                                    for child in best_nodes[nr:]
                                    if not self.is_in_line(node, other_bound)
                                ]
                                bound[-1].children = children
                                return best_node
                            else:
                                forbidden.append(best_node)
                                break

                        elif (
                            nr + 1 >= len(best_nodes) and len(node.parent.children) > 1
                        ):
                            # Update other_bound by adding next node and append forbidden nodes
                            other_bound[:] = other_bound[:i]
                            other_bound[-1].children.pop(0)
                            other_bound.append(other_bound[-1].children[0])
                            forbidden_other.append(node)

                            # Exclude nodes that already belong to other_bound
                            children = [
                                child
                                for child in best_nodes[nr:]
                                if not self.is_in_line(node, other_bound)
                            ]
                            bound[-1].children = children
                            return best_node

                        elif (
                            nr + 1 < len(best_nodes) and len(node.parent.children) == 1
                        ):
                            forbidden.append(best_node)
                            break
                        else:
                            return None
            else:
                # Remove nodes already in other_bound from children and return best_node
                children = [
                    node
                    for node in best_nodes[nr:]
                    if not self.is_in_line(node, other_bound)
                ]
                bound[-1].children = children
                return best_node

        return None

    def create_node(self, parent, cone, first=False):
        """
        Creates a new node based on the given parent node and cone position.

        Args:
            parent (Node): The parent node from which the new node will be created.
            cone (tuple): A tuple containing the x and y coordinates, color, and id of the cone.
            first (bool, optional): A flag indicating if this is the first node. Defaults to False.

        Returns:
            Node: The newly created node if the distance and turn constraints are satisfied, otherwise None.

        """
        dist = self.distance(parent, cone)
        if dist < self.max_dist and dist > self.min_dist:
            angle = np.arctan2(cone[1] - parent.y, cone[0] - parent.x)

            turn = angle - parent.angle
            turn = copysign(
                min(abs(turn), 2 * np.pi - abs(turn)),
                turn,
            )
            if first:
                max_turn = self.max_turn * 3 / 2  # Increase max_turn for first node
            else:
                max_turn = self.max_turn
            if abs(turn) < max_turn:
                if first:
                    turn_change = 0
                else:
                    turn_change = abs(turn - parent.turn)
                cost = self.get_cost(dist, turn, turn_change)
                node = Node(
                    x=cone[0],
                    y=cone[1],
                    color=cone[2],
                    distance=dist,
                    parent=parent,
                    children=[],
                    angle=angle,
                    turn=turn,
                    cost=cost,
                    id=cone[3],
                )
                return node
        return None

    def get_cost(self, dist, turn, turn_change):
        """
        Calculate the cost based on distance, turn, and turn change factors.

        Parameters:
        dist (float): The distance factor.
        turn (float): The turn factor.
        turn_change (float): The turn change factor.

        Returns:
        float: The calculated cost.
        """
        return (
            self.dist_factor * dist
            + self.turn_factor * abs(turn)
            + self.turn_change_factor * turn_change
        )

    def is_in_line(self, cone, line):
        """
        Check if a cone is in a given line.

        Args:
            cone (Union[Node, Tuple[float, float]]): The cone to check. It can be an instance of Node or a tuple representing coordinates (x, y).
            line (List[Node]): The line to check against, represented as a list of Node instances.

        Returns:
            bool: True if the cone is in the line, False otherwise.
        """
        if isinstance(cone, Node):
            return any(node.x == cone.x and node.y == cone.y for node in line)
        else:
            return any(node.x == cone[0] and node.y == cone[1] for node in line)

    def distance(self, node1, node2):
        """
        Calculate the squared Euclidean distance between two nodes.

        Parameters:
        node1 (Node or tuple): The first node, which can be an instance of the Node class or a tuple containing (x, y) coordinates.
        node2 (Node or tuple): The second node, which can be an instance of the Node class or a tuple containing (x, y) coordinates.

        Returns:
        float: The squared Euclidean distance between node1 and node2.
        """
        if isinstance(node1, Node):
            x1, y1 = node1.x, node1.y
        else:
            x1, y1 = node1[0], node1[1]
        if isinstance(node2, Node):
            x2, y2 = node2.x, node2.y
        else:
            x2, y2 = node2[0], node2[1]
        return (x1 - x2) ** 2 + (y1 - y2) ** 2
