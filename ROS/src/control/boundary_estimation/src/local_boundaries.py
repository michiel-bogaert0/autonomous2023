from math import copysign

import numpy as np
import rospy
from node import Node


class LocalBoundaries:
    def __init__(self):
        self.max_dist = rospy.get_param("~max_distance", 36)
        self.min_dist = rospy.get_param("~min_distance", 0.5)
        self.dist_factor = rospy.get_param("~dist_factor", 1)
        self.turn_factor = rospy.get_param("~turn_factor", 1)
        self.turn_change_factor = rospy.get_param("~turn_change_factor", 1)
        self.color_indep = rospy.get_param("~use_color", False)
        self.bound_len = rospy.get_param("~local_bound_len", 6)
        self.strategy = rospy.get_param("~middle_line_strategy", "distance")

    def get_boundaries(self, cones):
        self.cones = cones
        self.left_bound = []
        self.right_bound = []
        self.left_forbidden = []
        self.right_forbidden = []

        root_node = Node(0, 0, 2, 0, None, [], 0, 0, 0, None)
        self.left_bound.append(root_node)
        self.right_bound.append(root_node)
        closest_left_node = self.get_closest_node(root_node, "left")
        self.left_bound.append(closest_left_node)
        closest_right_node = self.get_closest_node(root_node, "right")
        self.right_bound.append(closest_right_node)

        if closest_left_node is None or closest_right_node is None:
            return None, None
        it = 0

        while it < self.bound_len and len(self.left_bound) + len(
            self.right_bound
        ) - 2 < len(self.cones):
            left_first = False
            right_first = False
            if len(self.left_bound) == 2:
                left_first = True
            if len(self.right_bound) == 2:
                right_first = True
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

            if cross_angle_left > cross_angle_right:
                next_left_node = self.get_next_node("left", left_first)
                if next_left_node is not None:
                    self.left_bound.append(next_left_node)

            if cross_angle_right >= cross_angle_left or next_left_node is None:
                next_right_node = self.get_next_node("right", right_first)
                if next_right_node is not None:
                    self.right_bound.append(next_right_node)

                else:
                    break
            it += 1

        if self.left_bound == [] or self.right_bound == []:
            return None, None
        return self.left_bound, self.right_bound

    def get_closest_node(self, root_node, side):
        closest_cone = None
        closest_dist = float("inf")

        for cone in self.cones:
            dist = self.distance(root_node, cone)
            angle = np.arctan2(cone[1] - root_node.y, cone[0] - root_node.x)

            if side == "left" and angle > 0 and angle < np.pi / 2:
                if dist < closest_dist:
                    closest_dist = dist
                    closest_cone = cone

            elif side == "right" and angle < 0 and angle > -np.pi / 2:
                if dist < closest_dist:
                    closest_dist = dist
                    closest_cone = cone
        if closest_cone is None:
            return None
        closest_node = Node(
            closest_cone[0],
            closest_cone[1],
            closest_cone[2],
            closest_dist,
            root_node,
            [],
            0,
            0,
            0,
            closest_cone[3],
        )
        root_node.children.append(closest_node)
        return closest_node

    def get_next_node(self, side, first=False):
        if side == "left":
            bound = self.left_bound
            other_bound = self.right_bound
            forbidden = self.left_forbidden
            forbidden_other = self.right_forbidden
        elif side == "right":
            bound = self.right_bound
            other_bound = self.left_bound
            forbidden = self.right_forbidden
            forbidden_other = self.left_forbidden

        nodes = []

        for cone in self.cones:
            if not self.is_in_line(cone, bound) and not self.is_in_line(
                cone, forbidden
            ):
                node = self.create_node(bound[-1], cone, first)
                if node is not None:
                    nodes.append(node)

        # sorteer op toenemende cost
        best_nodes = sorted(nodes, key=lambda node: node.cost)

        for nr, best_node in enumerate(best_nodes):
            if self.is_in_line(best_node, other_bound):
                for i, node in enumerate(other_bound):
                    if node.x == best_node.x and node.y == best_node.y:
                        node.parent.children = [
                            child
                            for child in node.parent.children
                            if not self.is_in_line(child, bound)
                        ]
                        if nr + 1 < len(best_nodes) and len(node.parent.children) > 1:
                            if (
                                node.parent.children[1].cost < best_nodes[nr + 1].cost
                                and self.color_indep
                            ) or (
                                (
                                    side == "left"
                                    and best_node.color == 0
                                    or side == "right"
                                    and best_node.color == 1
                                )
                                and not self.color_indep
                            ):
                                other_bound[:] = other_bound[:i]
                                other_bound[-1].children.pop(0)
                                other_bound.append(other_bound[-1].children[0])

                                forbidden_other.append(node)
                                # we halen de nodes die al in de andere lijn zitten eruit
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
                            other_bound[:] = other_bound[:i]
                            other_bound[-1].children.pop(0)
                            other_bound.append(other_bound[-1].children[0])
                            forbidden_other.append(node)
                            # we halen de nodes die al in de andere lijn zitten eruit
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
                # we halen de nodes die al in de andere lijn zitten eruit
                children = [
                    node
                    for node in best_nodes[nr:]
                    if not self.is_in_line(node, other_bound)
                ]
                bound[-1].children = children
                return best_node

        return None

    def create_node(self, parent, cone, first=False):
        dist = self.distance(parent, cone)
        if dist < self.max_dist and dist > self.min_dist:
            angle = np.arctan2(cone[1] - parent.y, cone[0] - parent.x)

            turn = angle - parent.angle
            turn = copysign(
                min(abs(turn), 2 * np.pi - abs(turn)),
                turn,
            )
            max_turn = np.pi / 2
            if first:
                max_turn = 3 * np.pi / 4
            if abs(turn) < max_turn:
                if first:
                    turn_change = 0
                else:
                    turn_change = abs(turn - parent.turn)
                cost = self.get_cost(dist, turn, turn_change)
                node = Node(
                    cone[0],
                    cone[1],
                    cone[2],
                    dist,
                    parent,
                    [],
                    angle,
                    turn,
                    cost,
                    cone[3],
                )
                return node
        return None

    def get_cost(self, dist, turn, turn_change):
        return (
            self.dist_factor * dist
            + self.turn_factor * abs(turn)
            + self.turn_change_factor * turn_change
        )

    def is_in_line(self, cone, line):
        if isinstance(cone, Node):
            return any(node.x == cone.x and node.y == cone.y for node in line)
        else:
            return any(node.x == cone[0] and node.y == cone[1] for node in line)

    def distance(self, node1, node2):
        """Calculates squared distance between two nodes/cones"""
        if isinstance(node1, Node):
            x1, y1 = node1.x, node1.y
        else:
            x1, y1 = node1[0], node1[1]
        if isinstance(node2, Node):
            x2, y2 = node2.x, node2.y
        else:
            x2, y2 = node2[0], node2[1]
        return (x1 - x2) ** 2 + (y1 - y2) ** 2
