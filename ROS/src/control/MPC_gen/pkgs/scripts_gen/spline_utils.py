import matplotlib.pyplot as plt
import numpy as np
from optimal_control_gen.Bspline import Bspline


def create_spline(arr, color, derivative=False, plot=False):
    degree = 2
    kk = np.linspace(0, 1, len(arr) - degree + 1)

    bspline = Bspline(kk, degree)

    control_points = bspline.compute_control_points(np.array(arr))

    curve = bspline.gen_curve(control_points)
    spline_arr = curve(np.linspace(0, 1, 1000))

    if plot:
        plt.plot(control_points[:, 0], control_points[:, 1], "o", c=color)
        plt.plot([x[0] for x in arr], [x[1] for x in arr], "--", c=color)
        plt.plot(spline_arr[:, 0], spline_arr[:, 1], c=color)

    if derivative:
        dcurve = curve.derivative(o=1)
        der_arr = dcurve(np.linspace(0, 1, 1000))

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
