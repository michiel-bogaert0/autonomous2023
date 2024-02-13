import numpy as np
from BE_dc.node import Node


def get_color_lines(cones, header):
    max_dist = np.power(6, 2)
    min_dist = np.power(1, 2)
    max_angle_change = np.pi / 2

    blue_line = []
    yellow_line = []

    blue_cones = []
    yellow_cones = []
    for cone in cones:
        if cone[2] == 0:
            blue_cones.append(cone)
        elif cone[2] == 1:
            yellow_cones.append(cone)

    # TODO: It might be necessary to also put the orange cones in the lists, in order to bridge the gap at the finish

    # node at position of the car, distance 0, angle 0
    # children NOT IMPLEMENTED
    root_node = Node(0, 0, 0, None, [], 0, 0)

    # Create the blue line
    blue_line.append(root_node)

    # Get closest blue cone to the car
    while True:
        next_cone = get_next_node(
            blue_cones, blue_line, max_dist, min_dist, max_angle_change
        )

        if next_cone is None:
            break

        blue_line.append(next_cone)

    # Create the yellow line
    yellow_line.append(root_node)

    # Get closest yellow cone to the car
    while True:
        next_cone = get_next_node(
            yellow_cones, yellow_line, max_dist, min_dist, max_angle_change
        )

        if next_cone is None:
            break

        yellow_line.append(next_cone)

    return blue_line, yellow_line


def get_next_node(cones, line, max_dist, min_dist=0, max_angle_change=np.pi):
    # TODO: The next improvement will be to try out the different possible cones,
    # and for example choose the most restricting cone when two cones are near
    # eachother, and also take into account the consequence of choosing a cone:
    # bad cones may lead to a dead end.

    best_node = None

    smallest_angle_change = max_angle_change
    angle_change = 0.0
    angle = 0.0

    for cone in cones:
        is_in_line = any(node.x == cone[0] and node.y == cone[1] for node in line)

        if not is_in_line:
            dist = np.power(line[-1].x - cone[0], 2) + np.power(line[-1].y - cone[1], 2)

            if dist < max_dist and dist > min_dist:
                angle = np.arctan2(cone[1] - line[-1].y, cone[0] - line[-1].x)
                angle_change = abs(angle - line[-1].angle)
                angle_change = min(angle_change, 2 * np.pi - angle_change)

                if angle_change < smallest_angle_change:
                    smallest_angle_change = angle_change
                    if len(line) == 1:
                        best_node = Node(cone[0], cone[1], dist, line[-1], [], 0, 0)
                    else:
                        best_node = Node(
                            cone[0], cone[1], dist, line[-1], [], angle, angle_change
                        )

    return best_node
