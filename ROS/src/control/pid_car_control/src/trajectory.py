import numpy as np


class Trajectory:
    """
    Helper class to calculate the cars target point based on a given path it has to follow
    """

    def __init__(self, minimal_distance=2, t_step=0.1, max_angle=1):
        self.path = np.array([0, 0])
        self.minimal_distance = minimal_distance**2

        self.t_step = t_step
        self.max_angle = max_angle

        self.piecewise_equation = PiecewiseParametricEquation()

    def set_path(self, points):
        """
        Sets the internal path with new points. Expects a numpy array of shape (N, 2)
        It also converts it to a piecewise parametric equation, with t in [0, N)

        Args:
            points: (N,2) numpy array
        """

        self.piecewise_equation.clear()
        self.path = points

        # Make a parametric equation based on the path
        for i in range(len(points)):
            if i == 0:
                continue

            # Make parametric equation based on this point and the previous one
            p0 = points[i - 1]
            p1 = points[i]

            # It is like this to prevent some issues, see https://stackoverflow.com/questions/452610/how-do-i-create-a-list-of-python-lambdas-in-a-list-comprehension-for-loop
            def parametric_equation(p0, p1):
                return lambda t: (1 - t) * p0 + t * p1

            # Add them to the piecewise equation
            self.piecewise_equation += [
                parametric_equation(p0[0], p1[0]),
                parametric_equation(p0[1], p1[1]),
            ]

    def calculate_target_point(self, current_pos, current_angle):
        """
        Calculates a target point by traversing the path from start to finish.
        Returns the first points that matches the conditions given by minimal_distance and max_angle

        When no target point was found it returns the first point in the path (t=0)

        Args:
            current_pos: current position of the car
            current_angle: current heading of the car

        Returns:
            x {float}: x position of target point
            y {float}: y position of target point
            success {bool}: True when target point was found
        """

        # Iterate through t
        for t in np.arange(0.0, len(self.piecewise_equation), self.t_step):

            x, y = self.piecewise_equation(t)

            distance = (x - current_pos[0]) ** 2 + (y - current_pos[1]) ** 2
            if distance < self.minimal_distance:
                continue

            angle = (np.arctan2(y - current_pos[1], x - current_pos[0]) + np.pi / 2) % (2 * np.pi) - np.pi / 2

            if np.abs(angle - current_angle) > self.max_angle:
                continue

            return x, y, True

        # When nothing was found, return first point
        x, y = self.piecewise_equation(0)
        return x, y, False


class PiecewiseParametricEquation:
    """
    Helper class to make a piecewise, lineair parametric equation.
    """

    def __init__(self):
        self.piecewise_equation = []  # Contains [x(t), y(t)]

    def __len__(self):
        return len(self.piecewise_equation)

    def clear(self):
        self.piecewise_equation = []

    def __iadd__(self, subeq):
        """
        Adds a new parametric equation to itself

        Args:
            subeq: Must be a list of two (lambda) functions, evaluated with a single parameter, eg. [x(t), y(t)] with t in [0, 1]
        """
        self.piecewise_equation.append(subeq)
        return self

    def __call__(self, t):
        """
        Evaluates the correct equation based on t and returns the result

        Args:
            t: the value to evaluate the piecewise equation in. This parameter t must be in [0, N], with N the number of lineair segments
                Every segment has a range of t in [0, 1], so t gets transformed.

            Example:
            A t = 3.6 will evaluate the fourth section in t = 0.6

        Returns:
            The evaluated x, y position
        """

        index = int(t // 1)
        remaining_t = t % 1

        if len(self) == 0:
            return 0, 0

        return (
            self.piecewise_equation[index][0](remaining_t),
            self.piecewise_equation[index][1](remaining_t),
        )
