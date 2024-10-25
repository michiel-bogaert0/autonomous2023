import numpy as np
import rospy


class DistancePath:
    def __init__(self):
        self.current_left_cone = None
        self.current_right_cone = None
        self.local = rospy.get_param("~local", True)

    def get_path(self, left_bound, right_bound):
        self.left_bound = left_bound[1:]
        self.right_bound = right_bound[1:]
        self.left_bound_copy = left_bound[1:]
        self.right_bound_copy = right_bound[1:]
        self.middle_line = []

        if self.left_bound != []:
            self.current_left_cone = self.left_bound.pop(0)
        if self.right_bound != []:
            self.current_right_cone = self.right_bound.pop(0)
        if self.current_left_cone is not None and self.current_right_cone is not None:
            self.add_midpoint_to_path()

        while self.left_bound != [] and self.right_bound != []:
            distance_from_current_left_cone = self.distance(
                self.current_left_cone, self.right_bound[0]
            )
            distance_from_current_right_cone = self.distance(
                self.left_bound[0], self.current_right_cone
            )

            if distance_from_current_left_cone <= distance_from_current_right_cone:
                self.current_right_cone = self.right_bound.pop(0)
                self.add_midpoint_to_path()
            else:
                self.current_left_cone = self.left_bound.pop(0)
                self.add_midpoint_to_path()

        if self.local:
            middle_line = []
            for tupel in self.middle_line:
                id0 = tupel[0]
                id1 = tupel[1]
                node0 = next(node for node in self.left_bound_copy if node.id == id0)
                node1 = next(node for node in self.right_bound_copy if node.id == id1)
                middle_line.append(
                    (
                        (node0.x + node1.x) / 2,
                        (node0.y + node1.y) / 2,
                        node0.id,
                        node1.id,
                    )
                )
            self.middle_line = middle_line

        return self.middle_line

    def distance(self, left_cone, right_cone):
        dist = np.power(left_cone.x - right_cone.x, 2) + np.power(
            left_cone.y - right_cone.y, 2
        )
        return dist

    def add_midpoint_to_path(self):
        self.middle_line.append((self.current_left_cone.id, self.current_right_cone.id))
