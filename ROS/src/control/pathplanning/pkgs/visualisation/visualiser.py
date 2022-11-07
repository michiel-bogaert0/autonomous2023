from turtle import color
import matplotlib.pyplot as plt
import numpy as np
from typing import List


class Visualiser:

    def __init__(self, figure):
        self.fig = figure

    def clear(self):
        self.fig.clf()

        # Zoom in to get a better view
        plt.xlim((-5, 15))
        plt.ylim((-10, 10))

    def draw(self):
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def visualise(self, cones: np.ndarray, viz_cones: np.ndarray, edge_centers: np.ndarray, root: "Node",
                  branch: np.ndarray, center_line : np.ndarray = None):
        """
        Visualise the current planning situation

        Args:
            cones: All the cones of the track (non visible ones will be in grey)
            viz_cones: The visible cones
            edge_centers: The centers between cones
            root: The root object of the path planning tree
            branch: The best branch found
            center_line: (Optional) The ground truth center line
        """

        self.clear()

        self.draw_cones(cones, viz_cones)
        self.draw_edge_centers(edge_centers)
        self.draw_car()

        nodes = self.draw_nodes_and_leaves(root)
        self.draw_racing_lines(nodes)
        self.draw_branch(branch)

        # Draw center line
        #for i in range(len(center_line) - 1):
        #    plt.plot([center_line[i][0], center_line[i + 1][0]], [center_line[i][1], center_line[i + 1][1]], c='grey', linewidth=1,
        #             linestyle='solid')

        self.draw()

    def draw_cones(self, cones: np.ndarray, viz_cones: np.ndarray):
        """
        Draws the cones to the screen. Visible ones are coloured, the rest is grey.

        Args:
            cones: All existing cones
            viz_cones: The cones visible by the car
        """
        grey = cones[:, :-1]
        blue = viz_cones[np.where(viz_cones[:, 2] == 0)][:, :-1]
        yellow = viz_cones[np.where(viz_cones[:, 2] == 1)][:, :-1]

        if len(grey) > 0:
            plt.scatter(grey[:, 0], grey[:, 1], c='grey', marker='o')

        if len(blue) > 0:
            plt.scatter(blue[:, 0], blue[:, 1], c='blue', marker='o')

        if len(yellow) > 0:
            plt.scatter(yellow[:, 0], yellow[:, 1], c='yellow', marker='o')

    def draw_safetydist(self, viz_cones: np.ndarray, safety_dist: float):
        """
        Draws the safety distance around cones as circles to the screen.

        Args:
            viz_cones: The cones visible by the car
            safety_dist: The safety distance around a car
        """
        ax = plt.gca()
        for c in viz_cones:
            ax.add_patch(plt.Circle((c[0], c[1]), safety_dist, color = 'r', alpha = 0.5, zorder=0))

    def draw_genAngle(self, angle: float):
        ax = plt.gca()
        ax.add_patch(plt.Polygon([[0,0],
                                  [0,20],
                                  [30*np.cos(angle),30*np.sin(angle)]],
                                 color = 'r',
                                 alpha = 0.5,
                                 zorder=0,
                                 linewidth=0))
        ax.add_patch(plt.Polygon([[0,0],
                                  [0,-20],
                                  [30*np.cos(-angle),30*np.sin(-angle)]],
                                 color = 'r',
                                 alpha = 0.5,
                                 zorder=0,
                                 linewidth=0))
        ax.add_patch(plt.Rectangle((-20,-20), 20, 40,
                                   color = 'r',
                                   alpha = 0.5,
                                   zorder=0,
                                   linewidth=0))

    def draw_edge_centers(self, edge_centers: np.ndarray):
        if len(edge_centers) > 0:
            plt.scatter(edge_centers[:, 0], edge_centers[:, 1], c='purple', marker='x')

    def draw_nodes_and_leaves(self, root) -> np.ndarray:
        stack = [root]
        nodes = []

        while stack:
            node = stack.pop()
            if node in nodes:
                continue

            # Nodes and leaves are the same as edges for the center algorithm, so don't draw them
            # plt.scatter(node.x, node.y, c='black', marker='.')

            nodes.append(node)
            for child in node.children:
                stack.append(child)

        return nodes

    def draw_racing_lines(self, nodes: List["Node"]):
        for node in nodes:
            if node.parent is not None:
                parent = node.parent

                plt.plot([node.x, parent.x], [node.y, parent.y], c='black', linewidth=1, linestyle='dashed')

    def draw_branch(self, branch: np.ndarray):
        plt.plot([0, branch[0].x], [0, branch[0].y], c='green', linewidth=2,
                 linestyle='solid')

        for i in range(len(branch) - 1):
            plt.plot([branch[i].x, branch[i + 1].x], [branch[i].y, branch[i + 1].y], c='green', linewidth=2,
                     linestyle='solid')

    def draw_car(self):
        plt.scatter(0, 0, c='green', marker='s')

    def draw_children(self, node: "RttNode", points: np.ndarray):
        if len(node.children) > 0:
            l = []
            n = points[node.index]
            for child in node.children:
                c = points[child.index]
                x = [n[0], c[0]]
                y = [n[1], c[1]]
                l.append(x)
                l.append(y)
            plt.plot(*l, c='gray', linewidth=1, linestyle='solid', marker=".", mec='pink', mew=1)

    def draw_tree(self, root: "RttNode", points: np.ndarray):
        self.draw_children(root, points)
        for child in root.children:
            self.draw_tree(child, points)

