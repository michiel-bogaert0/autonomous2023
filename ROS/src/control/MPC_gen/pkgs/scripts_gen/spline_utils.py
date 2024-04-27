import casadi as cd
import matplotlib.pyplot as plt
import numpy as np
from optimal_control_gen.Bspline import Bspline


def create_spline(arr, color, derivative=False, derivative2=False, plot=False):
    print(derivative2)
    degree = 2
    kk = np.linspace(0, 1, len(arr) - degree + 1)

    bspline = Bspline(kk, degree)

    control_points = bspline.compute_control_points(np.array(arr))

    curve = bspline.gen_curve(control_points)
    spline_arr = curve(np.linspace(0, 1, 1000))

    if plot:
        # plt.plot(control_points[:, 0], control_points[:, 1], "o", c=color)
        plt.plot([x[0] for x in arr], [x[1] for x in arr], "--", c=color)
        plt.plot(spline_arr[:, 0], spline_arr[:, 1], c=color)

    if derivative:
        dcurve = curve.derivative(o=1)
        der_arr = dcurve(np.linspace(0, 1, 1000))

        # normalize derivative vectors
        der_arr = der_arr / np.linalg.norm(der_arr, axis=1)[:, None]

        # Plot the derivative vectors originating from spline points
        for i in range(0, len(spline_arr), 1):
            plt.scatter(
                spline_arr[i, 0], spline_arr[i, 1], c=color, label="Spline Points"
            )
            plt.arrow(
                spline_arr[i, 0],
                spline_arr[i, 1],
                der_arr[i, 0],
                der_arr[i, 1],
                head_width=0.05,
                head_length=0.1,
                fc=color,
                ec=color,
                alpha=0.7,
            )

        if derivative2:
            ddcurve = curve.derivative(o=2)
            dder_arr = ddcurve(np.linspace(0, 1, 1000))

            # normalize derivative vectors
            dder_arr = dder_arr / np.linalg.norm(dder_arr, axis=1)[:, None]

            print(dder_arr)

            # Plot the derivative vectors originating from spline points
            for i in range(0, len(spline_arr), 1):
                plt.arrow(
                    spline_arr[i, 0] + der_arr[i, 0],
                    spline_arr[i, 1] + der_arr[i, 1],
                    dder_arr[i, 0],
                    dder_arr[i, 1],
                    head_width=0.05,
                    head_length=0.1,
                    fc="g",
                    ec="g",
                    alpha=0.7,
                )

        plt.show()

    return bspline, curve


def project_on_spline(points_on_spline, der_points, point, plot=False):
    """
    Project a point onto a spline
    Done by calculating the angle between the point and the derivative vector at each point on the spline
    And taking the one closest to 90 degrees
    """

    # Filter points on max distance
    idx_points = np.where(np.linalg.norm(points_on_spline - point, axis=1) < 4)[0]
    points_on_spline = points_on_spline[idx_points]
    der_points = der_points[idx_points]
    angles = []

    for point_on_spline, der_point in zip(points_on_spline, der_points):
        v_der = np.array([der_point[0], der_point[1]])
        v_point = np.array(
            [point[0] - point_on_spline[0], point[1] - point_on_spline[1]]
        )

        # calculate angle between the two vectors
        dot_prod = np.dot(v_point, v_der)
        magA = np.dot(v_der, v_der) ** 0.5
        magB = np.dot(v_point, v_point) ** 0.5

        angle = np.arccos(dot_prod / magB / magA)

        # Angle between 0 and 180
        if angle - np.pi >= 0:
            angle = 2 * np.pi - angle

        angles.append(angle)

    # Find angle closest to 90 degrees
    min_degree = np.argmin(np.abs(np.array(angles) - np.pi / 2))
    print(f"Angle closest to 90 degrees: {angles[min_degree] * 180 / np.pi}")

    angle_derivative = np.arctan2(
        der_points[min_degree, 1], der_points[min_degree, 0]
    )  # in radians
    print(angle_derivative * 180 / np.pi)

    if plot:
        # Car position
        plt.plot(point[0], point[1], "o", c="g")

        # Spline points
        # plt.plot(points_on_spline[:, 0], points_on_spline[:, 1], c="r")

        # Closest point on spline
        plt.plot(
            points_on_spline[min_degree, 0], points_on_spline[min_degree, 1], "o", c="b"
        )

        # Derivative vector
        plt.arrow(
            points_on_spline[min_degree, 0],
            points_on_spline[min_degree, 1],
            der_points[min_degree, 0],
            der_points[min_degree, 1],
            head_width=0.05,
            head_length=0.1,
            fc="b",
            ec="b",
            alpha=0.7,
        )

        # Vector from car position to closest point on spline
        plt.plot(
            [point[0], points_on_spline[min_degree, 0]],
            [point[1], points_on_spline[min_degree, 1]],
            c="g",
        )


def get_boundary_constraints(spline, tau, width, plot=False):
    """
    Get the boundary constraints for the spline
    """
    point = np.squeeze(spline(tau))

    # Get the derivative of the spline
    dcurve = spline.derivative(o=1)
    der_point = np.squeeze(dcurve(tau))

    tangent = np.array([-der_point[1], der_point[0]]) / np.linalg.norm(der_point)

    # TODO: make width dependent on tau
    pos_outer = point - width * tangent
    pos_inner = point + width * tangent

    track_constraint_lower = np.sum(tangent * pos_inner)
    track_constraint_upper = np.sum(tangent * pos_outer)

    print(np.sum(tangent * point))

    print(f"Track constraint lower: {track_constraint_lower}")
    print(f"Track constraint upper: {track_constraint_upper}")

    # get line through pos_inner
    slope_inner = -tangent[0] / tangent[1]
    intercept_inner = pos_inner[1] - slope_inner * pos_inner[0]

    # get line through pos_outer
    slope_outer = -tangent[0] / tangent[1]
    intercept_outer = pos_outer[1] - slope_outer * pos_outer[0]
    point_test = np.copy(point)
    point_test[0] -= 1.5
    point_test[1] += 1.5
    in_between = (slope_inner * point_test[0] + intercept_inner - point_test[1]) * (
        slope_outer * point_test[0] + intercept_outer - point_test[1]
    ) < 0
    print(in_between)

    if plot:
        plt.plot(point[0], point[1], "o", c="r")
        plt.plot(pos_inner[0], pos_inner[1], "o", c="b")
        plt.plot(pos_outer[0], pos_outer[1], "o", c="y")

        # generate x around point
        x = np.linspace(point[0] - 2, point[0] + 2, 100)
        y_inner = slope_inner * x + intercept_inner
        y_outer = slope_outer * x + intercept_outer
        plt.plot(x, y_inner, c="b")
        plt.plot(x, y_outer, c="y")

        plt.plot(point_test[0], point_test[1], "o", c="g")

    return slope_inner, intercept_inner, slope_outer, intercept_outer


def get_boundary_constraints_casadi(spline, dspline, taus, width, plot=False):
    """
    Get the boundary constraints for the spline
    """
    points = spline(taus)

    # Get the derivative of the spline
    dcurve = dspline
    der_points = dcurve(taus)

    # Add small noise to avoid division by zero
    der_points += 1e-10

    # Manually calculate norm to avoid some casadi error
    norms = cd.sqrt(der_points[:, 0] ** 2 + der_points[:, 1] ** 2)

    tangents = cd.horzcat(-der_points[:, 1], der_points[:, 0]) / norms

    pos_inner = points + width * tangents
    pos_outer = points - width * tangents

    slopes_inner = -tangents[:, 0] / (tangents[:, 1] + 1e-10)
    intercepts_inner = pos_inner[:, 1] - slopes_inner * pos_inner[:, 0]

    slopes_outer = slopes_inner
    intercepts_outer = pos_outer[:, 1] - slopes_outer * pos_outer[:, 0]

    if plot:
        plt.plot(points[:, 0], points[:, 1], "o", c="r")
        plt.plot(pos_inner[:, 0], pos_inner[:, 1], "o", c="b")
        plt.plot(pos_outer[:, 0], pos_outer[:, 1], "o", c="y")

        for i in range(len(points)):
            # generate x around point
            x = np.linspace(points[i, 0] - 2, points[i, 0] + 2, 100)
            y_inner = slopes_inner[i] * x + intercepts_inner[i]
            y_outer = slopes_outer[i] * x + intercepts_outer[i]
            plt.plot(x, y_inner, c="b")
            plt.plot(x, y_outer, c="y")

    return slopes_inner, intercepts_inner, slopes_outer, intercepts_outer
