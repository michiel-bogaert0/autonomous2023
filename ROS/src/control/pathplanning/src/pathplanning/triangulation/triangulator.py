import sys
import numpy as np
from pathplanning.triangulation.center_points import get_center_points
from pathplanning.triangulation.paths import TriangulationPaths
import pathplanning.triangulation.utils


class Triangulator:
    def __init__(
        self,
        triangulation_var_threshold: float,
        max_iter: int,
        plan_dist: float,
        max_angle_change: float,
        safety_dist: float,
    ) -> None:
        """Initialize triangulator

        Args:
            triangulation_var_threshold: Factor multiplied to the median of the variance of triangle lengths in order to filter bad triangles
            max_iter: Amount of iterations
            plan_dist: Maximum distance to plan path
            max_angle_change: Maximum angle change in path
            safety_dist: Safety distance from objects
        """
        self.triangulation_var_threshold = triangulation_var_threshold
        self.max_iter = max_iter
        self.plan_dist = plan_dist
        self.max_angle_change = max_angle_change
        self.safety_dist = safety_dist
        self.safety_dist_squared = safety_dist**2

        self.triangulation_paths = TriangulationPaths(max_iter, plan_dist, max_angle_change, safety_dist)

    def get_path(self, cones: np.ndarray):
        """Generate path based on the cones.

        Args:
            cones: Cones between which to generate path (array of [x, y, class])

        Returns:
            path
        """
        position_cones = cones[:, :-1]  # Extract the x and y positions

        # We need at least 4 cones for Delaunay triangulation
        if len(position_cones) < 4:
            return None

        # Perform triangulation and get the (useful) center points
        center_points, unique_center_points = get_center_points(position_cones, self.triangulation_var_threshold)

        _, leaves = self.triangulation_paths.get_all_paths(
            center_points, unique_center_points, cones[:, :-1]
        )

        path = self.get_best_path(leaves, cones)

        return path

    def get_best_path(self, leaves: list, cones: np.ndarray):
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

        # Iterate each leaf
        for i, leave in enumerate(leaves):

            # Find the path connecting this leaf to the root and reverse it
            path = []
            parent = leave
            while parent.parent is not None:
                path.append(parent)
                parent = parent.parent
            path.reverse()

            # Calculate the path cost and normalise it
            (
                color_cost,
                angle_cost,
                width_cost,
                spacing_cost,
                length_cost,
            ) = self.triangulation_paths.get_cost_branch(path, cones)
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

        # Get the least-cost path
        index = np.argmin(total_cost)
        paths = np.array(paths)

        return paths[index]
