import datetime
import math
import os

# import pathlib
import time
from typing import Union

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import quadprog
import rospy
from matplotlib.legend_handler import HandlerPatch
from mpl_toolkits.mplot3d import Axes3D
from scipy import spatial
from scipy.interpolate import interp1d, splev, splprep

########################################################################################################################
# ALMOST FINISHED FUNCTIONS:
# - interp_trajectory
# - B_spline_smoothing
# - calc_bound_dists
# - calc_distances_along_trajectory
# - prep_track
# - calc_ax_max_motors
# - calc_ggv
# - calc_splines
# - opt_min_curv


########################################################################################################################
def interp_trajectory(
    trajectory: np.ndarray,
    stepsize: float,
    interpolation_method: str = "quadratic",
) -> np.ndarray:
    """
    author:
    Kwinten Mortier

    .. description::
    Interpolate trajectory points to a new stepsize. Also works for boundaries.

    .. inputs::
    :param trajectory:              trajectory in the format [x, y, w_tr_left, w_tr_right] or [x, y].
    :type trajectory:               np.ndarray
    :param stepsize:                desired stepsize after interpolation in m.
    :type stepsize:                 float
    :param interpolation_method:    method used for interpolation (e.g. 'linear', 'quadratic', 'cubic').
    :type interpolation_method:     str

    .. outputs::
    :return trajectory_interp_cl:   interpolated trajectory [x, y, w_tr_left, w_tr_right] or [x, y].
    :rtype trajectory_interp_cl:    np.ndarray

    .. notes::
    Trajectory input is unclosed! Trajectory input must however be closable in the current form!
    """

    # Check inputs
    if trajectory.shape[1] != 2 and trajectory.shape[1] != 4:
        raise RuntimeError(
            "Trajectory input must be in the format [x, y] or [x, y, w_tr_left, w_tr_right]!"
        )
    if (trajectory[0] == trajectory[-1]).all():
        raise RuntimeError("Trajectory input must be unclosed!")

    # create closed trajectory
    trajectory_cl = np.vstack((trajectory, trajectory[0]))

    # calculate element lengths (euclidian distance)
    el_lengths_cl = np.sqrt(
        np.sum(np.power(np.diff(trajectory_cl[:, :2], axis=0), 2), axis=1)
    )

    # sum up total distance (from start) to every element
    dists_cum_cl = np.cumsum(el_lengths_cl)
    dists_cum_cl = np.insert(dists_cum_cl, 0, 0.0)

    # calculate desired lenghts depending on specified stepsize (+1 because last element is included)
    no_points_interp_cl = math.ceil(dists_cum_cl[-1] / stepsize) + 1
    dists_interp_cl = np.linspace(0.0, dists_cum_cl[-1], no_points_interp_cl)

    # interpolate closed track points
    trajectory_interp_cl = np.zeros((no_points_interp_cl, trajectory_cl.shape[1]))

    interpolator = interp1d(
        dists_cum_cl, trajectory_cl, kind=interpolation_method, axis=0
    )
    trajectory_interp_cl = interpolator(dists_interp_cl)

    return trajectory_interp_cl


def B_spline_smoothing(
    trajectory_cl: np.ndarray,
    smoothing_factor: float = 2.0,
) -> np.ndarray:
    """
    author:
    Kwinten Mortier

    .. description::
    Find a B-spline representation of the trajectory and smooth it in this process.

    .. inputs::
    :param trajectory_cl:               trajectory in the format [x, y, w_tr_left, w_tr_right] or [x, y].
    :type trajectory_cl:                np.ndarray
    :param smoothing_factor:            factor for smoothing the trajectory.
    :type smoothing_factor:             float

    .. outputs::
    :return trajectory_smoothed_cl:     smoothed trajectory [x, y].
    :rtype trajectory_smoothed_cl:      np.ndarray

    .. notes::
    Trajectory input and output are closed!
    """

    # Check inputs
    if trajectory_cl.shape[1] != 2:
        raise RuntimeError("Trajectory input must be in the format [x, y]!")
    if not (trajectory_cl[0] == trajectory_cl[-1]).all():
        raise RuntimeError("Trajectory input must be closed!")
    if smoothing_factor < 0.0:
        raise RuntimeError("Smoothing factor must be 0 or positive!")

    # Smooth path with BSpline interpolation
    tck, u = splprep(
        trajectory_cl.T, s=smoothing_factor, per=1
    )  # Transpose to get correct shape, splprep expects (2, N)

    trajectory_smoothed_cl = np.array(
        splev(u, tck)
    ).T  # Evaluate BSpline and transpose back to (N, 2)

    return trajectory_smoothed_cl


# TODO: Add calculation for error between old and new boundaries. Maybe average distance or something similar? Purely for validation purposes.
def calc_bound_dists(
    trajectory: np.ndarray,
    bound_left: np.ndarray,
    bound_right: np.ndarray,
    min_track_width: float = 3.0,
) -> np.ndarray:
    """
    author:
    Kwinten Mortier

    .. description::
    For every point on the reference trajectory, calculates the estimated perpendicular distance to the left and right boundaries. Determines which point on the boundary is closest to
    a point at a minimum distance to the trajectory and calculates the distance from the trajectory to that point. The distance is then stored in an array.

    .. inputs::
    :param trajectory:              array containing the reference trajectory [x, y] (Unclosed track!)
    :type trajectory:               np.ndarray
    :param bound_left:              array containing the left track boundary [x, y] (Unclosed boundary!)
    :type bound_left:               np.ndarray
    :param bound_right:             array containing the right track boundary [x, y] (Unclosed boundary!)
    :type bound_right:              np.ndarray
    :param min_track_width:         minimum track width in m
    :type min_track_width:          float

    .. outputs::
    :return bound_dists:            estimated perpendicular distance to boundaries along normal for every trajectory point (unclosed).
    :rtype bound_dists:             np.ndarray
    :return new_bound_left:         new left boundary after distance calculation (unclosed).
    :rtype new_bound_left:          np.ndarray
    :return new_bound_right:        new right boundary after distance calculation (unclosed).
    :rtype new_bound_right:         np.ndarray
    :return coeffs_x:               spline coefficients for the x coordinates of the closed trajectory.
    :rtype coeffs_x:                np.ndarray
    :return coeffs_y:               spline coefficients for the y coordinates of the closed trajectory.
    :rtype coeffs_y:                np.ndarray
    :return M:                      spline coefficients for the x and y coordinates of the closed trajectory.
    :rtype M:                       np.ndarray
    :return normvec_normalized:     normalized normal vectors to the trajectory.
    :rtype normvec_normalized:      np.ndarray

    .. notes::
    Make sure the boundaries contain sufficient amount of points to accurately estimate the distance
    Trajectory and boundaries must be unclosed!
    """

    # Check inputs
    if (trajectory[0] == trajectory[-1]).all():
        raise RuntimeError("Trajectory must be unclosed!")
    if (bound_left[0] == bound_left[-1]).all():
        raise RuntimeError("Left boundary must be unclosed!")
    if (bound_right[0] == bound_right[-1]).all():
        raise RuntimeError("Right boundary must be unclosed!")
    if (
        trajectory.shape[1] != 2
        or bound_left.shape[1] != 2
        or bound_right.shape[1] != 2
    ):
        raise RuntimeError("Trajectory and boundaries must be 2D arrays!")

    # Initialize an empty array to store the minimum distances
    min_dists = np.zeros(trajectory.shape)

    # Calculate the spline coefficients for the x and y coordinates of the closed trajectory
    trajectory_cl = np.vstack((trajectory, trajectory[0]))
    coeffs_x, coeffs_y, M, normvec_normalized = calc_splines(trajectory_cl)

    # Calculate the estimated perpendicular distance to the boundaries
    left_distance = min_track_width / 2
    right_distance = min_track_width / 2

    tree_left = spatial.KDTree(bound_left)
    tree_right = spatial.KDTree(bound_right)

    for i in range(trajectory.shape[0]):
        left_point = trajectory[i] - normvec_normalized[i] * left_distance
        right_point = trajectory[i] + normvec_normalized[i] * right_distance

        _, ind_left = tree_left.query(left_point, k=1)
        _, ind_right = tree_right.query(right_point, k=1)

        dist_left = math.sqrt(
            (trajectory[i][0] - bound_left[ind_left][0]) ** 2
            + (trajectory[i][1] - bound_left[ind_left][1]) ** 2
        )

        dist_right = math.sqrt(
            (trajectory[i][0] - bound_right[ind_right][0]) ** 2
            + (trajectory[i][1] - bound_right[ind_right][1]) ** 2
        )
        if i == 0:
            left_distance = dist_left
            right_distance = dist_right
        else:
            if dist_left <= left_distance:
                left_distance = dist_left - abs(dist_left - left_distance) / 2
            else:
                left_distance = dist_left + abs(dist_left - left_distance) / 2
            if dist_right <= right_distance:
                right_distance = dist_right - abs(dist_right - right_distance) / 2
            else:
                right_distance = dist_right + abs(dist_right - right_distance) / 2

        min_dists[i] = np.array([dist_left, dist_right])

    # New boundaries
    new_bound_left = trajectory - normvec_normalized * min_dists[:, 0].reshape(-1, 1)
    new_bound_right = trajectory + normvec_normalized * min_dists[:, 1].reshape(-1, 1)

    # Check spline normals for crossing points
    normals_crossing = check_normals_crossing(
        track=np.hstack((trajectory, min_dists)),
        normvec_normalized=normvec_normalized,
        horizon=10,
    )
    if normals_crossing:
        rospy.logerr(
            "At least two spline normalse are crossed! Check input or increase trajectory smoothing factor!"
        )

    # Enforce minimum track width
    manipulated_track_width = False

    for i in range(trajectory.shape[0]):
        track_width = min_dists[i, 0] + min_dists[i, 1]

        if track_width < min_track_width:
            manipulated_track_width = True
            rospy.loginfo(
                f"Track width too small at point {trajectory[i]}, track width was {track_width} m!"
            )

            # Inflate to both sides equally
            min_dists[i, 0] += (min_track_width - track_width) / 2
            min_dists[i, 1] += (min_track_width - track_width) / 2

            new_bound_left[i] = trajectory[i] - normvec_normalized[i] * min_dists[i, 0]
            new_bound_right[i] = trajectory[i] + normvec_normalized[i] * min_dists[i, 1]

    if manipulated_track_width:
        rospy.logerr("Track width was manipulated to enforce minimum track width!")

    return (
        min_dists,
        new_bound_left,
        new_bound_right,
        coeffs_x,
        coeffs_y,
        M,
        normvec_normalized,
    )


def calc_distances_along_trajectory(
    trajectory: np.ndarray,
) -> np.ndarray:
    """
    author:
    Kwinten Mortier

    .. description::
    Calculates the distances along a trajectory for every point.

    .. inputs::
    :param trajectory:           trajectory in the format [x, y] (unclosed).
    :type trajectory:            np.ndarray

    .. outputs::
    :return distances_along_traj:   distances along the trajectory for every point.
    :rtype distances_along_traj:    np.ndarray
    """
    # Check inputs
    if (trajectory[0] == trajectory[-1]).all():
        raise RuntimeError("Trajectory must be unclosed!")

    # Calculate distances along the closed trajectory
    distances_along_traj = np.zeros(trajectory.shape[0])
    for i in range(1, trajectory.shape[0]):
        distances_along_traj[i] = distances_along_traj[i - 1] + np.linalg.norm(
            trajectory[i] - trajectory[i - 1]
        )

    return distances_along_traj


def prep_track(
    trajectory: np.ndarray,
    cones_left: np.ndarray,
    cones_right: np.ndarray,
    min_track_width: float = 3.0,
    stepsize_trajectory: float = 0.5,
    stepsize_boundaries: float = 0.1,
    sf_trajectory: float = 2.0,
    sf_boundaries: float = 0.1,
    debug_info: bool = False,
    plot_prep: bool = False,
) -> tuple:
    """
    author:
    Kwinten Mortier

    .. description::
    This function prepares the track for the IQP minimum curvature optimization.
    It interpolates the trajectory to a fixed stepsize and smooths the trajectory using a B-spline.
    It creates boundaries from cones by interpolating to a fixed stepsize and smoothing the boundaries using a B-spline.
    It calculates the perpendicular distance between the trajectory and the boundaries.
    Finally, it returns the prepared track in the format [x, y, w_tr_right, w_tr_left].

    .. inputs::
    :param trajectory:                  array containing the reference trajectory [x, y] (Unclosed track!)
    :type trajectory:                   np.ndarray
    :param cones_left:                  array containing the left track cones [x, y] (Unclosed!)
    :type cones_left:                   np.ndarray
    :param cones_right:                 array containing the right track cones [x, y] (Unclosed!)
    :type cones_right:                  np.ndarray
    :param min_track_width:             minimum track width in m
    :type min_track_width:              float
    :param stepsize_trajectory:         desired stepsize after interpolation in m for the trajectory
    :type stepsize_trajectory:          float
    :param stepsize_boundaries:         desired stepsize after interpolation in m for the boundaries
    :type stepsize_boundaries:          float
    :param sf_trajectory:               factor for smoothing the trajectory and boundaries.
    :type sf_trajectory:                float
    :param sf_boundaries:               factor for smoothing the trajectory and boundaries.
    :type sf_boundaries:                float
    :param debug_info:                  flag to show debugging information
    :type debug_info:                   bool
    :param plot_prep:                   flag to show the preprocessing steps on the map
    :type plot_prep:                    bool

    .. outputs::
    :return prepped_track:              preprocessed track for the IQP optimization [x, y, w_tr_left, w_tr_right] (unclosed).
    :rtype prepped_track:               np.ndarray
    :return bound_left:                 left boundary (unclosed).
    :rtype bound_left:                  np.ndarray
    :return bound_right:                right boundary (unclosed).
    :rtype bound_right:                 np.ndarray
    :return coeffs_x:                   spline coefficients for the x coordinates of the closed trajectory.
    :rtype coeffs_x:                    np.ndarray
    :return coeffs_y:                   spline coefficients for the y coordinates of the closed trajectory.
    :rtype coeffs_y:                    np.ndarray
    :return M:                          spline coefficients for the x and y coordinates of the closed trajectory.
    :rtype M:                           np.ndarray
    :return normvec_normalized:         normalized normal vectors to the preprocessed trajectory.
    :rtype normvec_normalized:          np.ndarray
    :return psi_prepped_track:          heading of the preprocessed trajectory.
    :rtype psi_prepped_track:           np.ndarray
    :return kappa_prepped_track:        curvature of the preprocessed trajectory.
    :rtype kappa_prepped_track:         np.ndarray
    :return dkappa_prepped_track:       curvature derivative of the preprocessed trajectory.
    :rtype dkappa_prepped_track:        np.ndarray

    .. notes::
    Make sure the boundaries contain sufficient amount of points to accurately estimate the distance
    Input trajectory and cones must be unclosed!
    """

    # Check inputs
    if (
        (trajectory[0] == trajectory[-1]).all()
        or (cones_left[0] == cones_left[-1]).all()
        or (cones_right[0] == cones_right[-1]).all()
    ):
        raise RuntimeError("Trajectory and cones must be unclosed!")
    if (
        trajectory.shape[1] != 2
        or cones_left.shape[1] != 2
        or cones_right.shape[1] != 2
    ):
        raise RuntimeError("Trajectory and cones must be 2D arrays!")

    # Interpolate trajectory
    trajectory_interp_cl = interp_trajectory(
        trajectory=trajectory,
        stepsize=stepsize_trajectory,
        interpolation_method="quadratic",
    )

    # Smooth trajectory
    trajectory_smoothed_cl = B_spline_smoothing(
        trajectory_cl=trajectory_interp_cl, smoothing_factor=sf_trajectory
    )
    trajectory_smoothed = trajectory_smoothed_cl[:-1]

    # Interpolate boundaries
    bound_left_interp_cl = interp_trajectory(
        trajectory=cones_left,
        stepsize=stepsize_boundaries,
        interpolation_method="quadratic",
    )
    bound_right_interp_cl = interp_trajectory(
        trajectory=cones_right,
        stepsize=stepsize_boundaries,
        interpolation_method="quadratic",
    )

    # Smooth boundaries
    bound_left_smoothed_cl = B_spline_smoothing(
        trajectory_cl=bound_left_interp_cl, smoothing_factor=sf_boundaries
    )
    bound_right_smoothed_cl = B_spline_smoothing(
        trajectory_cl=bound_right_interp_cl, smoothing_factor=sf_boundaries
    )
    bound_left_smoothed = bound_left_smoothed_cl[:-1]
    bound_right_smoothed = bound_right_smoothed_cl[:-1]

    # Calculate distances to boundaries
    (
        bound_dists,
        new_bound_left,
        new_bound_right,
        coeffs_x,
        coeffs_y,
        M,
        normvec_normalized,
    ) = calc_bound_dists(
        trajectory=trajectory_smoothed,
        bound_left=bound_left_smoothed,
        bound_right=bound_right_smoothed,
        min_track_width=min_track_width,
    )

    track = np.hstack((trajectory_smoothed, bound_dists))

    new_bound_left_cl = np.vstack((new_bound_left, new_bound_left[0]))
    new_bound_right_cl = np.vstack((new_bound_right, new_bound_right[0]))

    prepped_track = track
    bound_left = new_bound_left
    bound_right = new_bound_right

    # Calculate the heading psi, the curvature kappa and the curvature derivative d_kappa of the preprocessed track
    psi_prepped_track, kappa_prepped_track, dkappa_prepped_track = calc_head_curv_an(
        coeffs_x=coeffs_x,
        coeffs_y=coeffs_y,
        ind_spls=np.arange(prepped_track.shape[0]),
        t_spls=np.zeros(prepped_track.shape[0]),
        calc_curv=True,
        calc_dcurv=True,
    )

    # Calculate the distances along the closed trajectory
    distances_along_traj = calc_distances_along_trajectory(
        trajectory=prepped_track[:, :2],
    )

    ########################################################################################################################
    # Preprocessing plots:
    # - Initial data
    # - Quadratic interpolation
    # - BSpline smoothing
    # - Boundary distance estimation
    # - Preprocessed track
    # - Curvature preprocessed track 2D
    # - Curvature preprocessed track 3D
    # - Curvature derivative preprocessed track

    ########################################################################################################################
    if plot_prep:
        # Plotting variables
        trajectory_plot = np.vstack((trajectory, trajectory[0]))
        cones_left_plot = cones_left
        cones_right_plot = cones_right

        trajectory_interp_plot = trajectory_interp_cl
        bound_left_interp_plot = bound_left_interp_cl
        bound_right_interp_plot = bound_right_interp_cl

        trajectory_smoothed_plot = trajectory_smoothed_cl
        bound_left_smoothed_plot = bound_left_smoothed_cl
        bound_right_smoothed_plot = bound_right_smoothed_cl

        new_bound_left_plot = new_bound_left_cl
        new_bound_right_plot = new_bound_right_cl

        kappa_prepped_track_plot = kappa_prepped_track
        distances_along_traj_plot = distances_along_traj
        kappa_prepped_track_plot3D = np.hstack(
            (kappa_prepped_track, kappa_prepped_track[0])
        )
        dkappa_prepped_track_plot = dkappa_prepped_track
        dkappa_prepped_track_plot3D = np.hstack(
            (dkappa_prepped_track, dkappa_prepped_track[0])
        )

        # Arrow variables
        point1_arrow = trajectory[0]
        point2_arrow = trajectory[1]
        vec_arrow = point2_arrow - point1_arrow

        # Folder path for preprocessing plots
        # folder_path = (
        #     pathlib.Path.home()
        #     / "autonomous2023/ROS/src/control/minimum_curvature/data/preprocessing"
        # )

        # ------------------------------------------------------------------------------------------------------------------
        # Initial data -----------------------------------------------------------------------------------------------------
        # - Plotting the reference trajectory (initial_data_1)
        # - Plotting the left cones (initial_data_2)
        # - Plotting the right cones (initial_data_3)
        # - Plotting the driving direction (initial_data_4)
        # ------------------------------------------------------------------------------------------------------------------
        plt.figure("Initial data")
        plt.title("Initial data")
        plt.grid()
        initial_data_1 = plt.plot(
            trajectory_plot[:, 0],
            trajectory_plot[:, 1],
            color=normalize_color((0, 0, 0)),
            linewidth=0.5,
            linestyle="dashed",
            marker="o",
            markersize=5,
            markerfacecolor=normalize_color((0, 0, 0)),
        )
        initial_data_2 = plt.scatter(
            cones_left_plot[:, 0],
            cones_left_plot[:, 1],
            color=normalize_color((0, 0, 255)),
            marker="o",
            s=25,
        )
        initial_data_3 = plt.scatter(
            cones_right_plot[:, 0],
            cones_right_plot[:, 1],
            color=normalize_color((230, 240, 0)),
            marker="o",
            s=25,
        )
        initial_data_4 = plt.arrow(
            point1_arrow[0],
            point1_arrow[1],
            vec_arrow[0],
            vec_arrow[1],
            width=0.3,
            head_width=1.0,
            head_length=1.0,
            fc="g",
            ec="g",
        )
        plt.legend(
            [initial_data_1[0], initial_data_2, initial_data_3, initial_data_4],
            ["Reference trajectory", "Left cones", "Right cones", "Driving direction"],
            handler_map={
                mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)
            },
        )
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.xlabel("x-distance from original car position in m")
        plt.ylabel("y-distance from original car position in m")

        plt.show()

        # save_plot_to_folder(plt, folder_path, "Initial Data")
        # plt.close()

        # ------------------------------------------------------------------------------------------------------------------
        # Quadratic interpolation ------------------------------------------------------------------------------------------
        # - Plotting the reference trajectory (quad_interp_1)
        # - Plotting the left cones (quad_interp_2)
        # - Plotting the right cones (quad_interp_3)
        # - Plotting the interpolated trajectory (quad_interp_4)
        # - Plotting the interpolated left boundary (quad_interp_5)
        # - Plotting the interpolated right boundary (quad_interp_6)
        # - Plotting the driving direction (quad_interp_7)
        # ------------------------------------------------------------------------------------------------------------------
        plt.figure("Quadratic interpolation")
        plt.title("Quadratic interpolation")
        plt.grid()
        quad_interp_1 = plt.scatter(
            trajectory_plot[:, 0],
            trajectory_plot[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        # quad_interp_2
        plt.scatter(
            cones_left_plot[:, 0],
            cones_left_plot[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        # quad_interp_3
        plt.scatter(
            cones_right_plot[:, 0],
            cones_right_plot[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        quad_interp_4 = plt.plot(
            trajectory_interp_plot[:, 0],
            trajectory_interp_plot[:, 1],
            color=normalize_color((0, 0, 0)),
            linewidth=0.5,
            linestyle="dashed",
            marker="o",
            markersize=3,
            markerfacecolor=normalize_color((0, 0, 0)),
        )
        quad_interp_5 = plt.plot(
            bound_left_interp_plot[:, 0],
            bound_left_interp_plot[:, 1],
            color=normalize_color((0, 0, 255)),
            linewidth=0.5,
            linestyle="dashed",
            marker="o",
            markersize=3,
            markerfacecolor=normalize_color((0, 0, 255)),
        )
        quad_interp_6 = plt.plot(
            bound_right_interp_plot[:, 0],
            bound_right_interp_plot[:, 1],
            color=normalize_color((230, 240, 0)),
            linewidth=0.5,
            linestyle="dashed",
            marker="o",
            markersize=3,
            markerfacecolor=normalize_color((230, 240, 0)),
        )
        quad_interp_7 = plt.arrow(
            point1_arrow[0],
            point1_arrow[1],
            vec_arrow[0],
            vec_arrow[1],
            width=0.3,
            head_width=1.0,
            head_length=1.0,
            fc="g",
            ec="g",
        )
        plt.legend(
            [
                quad_interp_1,
                quad_interp_4[0],
                quad_interp_5[0],
                quad_interp_6[0],
                quad_interp_7,
            ],
            [
                "Initial data",
                "Interpolated trajectory",
                "Interpolated left boundary",
                "Interpolated right boundary",
                "Driving direction",
            ],
            handler_map={
                mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)
            },
        )
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.xlabel("x-distance from original car position in m")
        plt.ylabel("y-distance from original car position in m")

        plt.show()

        # save_plot_to_folder(plt, folder_path, "Quadratic Interpolation")
        # plt.close()

        # ------------------------------------------------------------------------------------------------------------------
        # BSpline smoothing ------------------------------------------------------------------------------------------------
        # - Plotting the reference trajectory (smoothing_1)
        # - Plotting the left cones (smoothing_2)
        # - Plotting the right cones (smoothing_3)
        # - Plotting the interpolated trajectory (smoothing_4)
        # - Plotting the interpolated left boundary (smoothing_5)
        # - Plotting the interpolated right boundary (smoothing_6)
        # - Plotting the smoothed trajectory (smoothing_7)
        # - Plotting the smoothed left boundary (smoothing_8)
        # - Plotting the smoothed right boundary (smoothing_9)
        # - Plotting the driving direction (smoothing_10)
        # ------------------------------------------------------------------------------------------------------------------
        plt.figure("BSpline smoothing")
        plt.title("BSpline smoothing")
        plt.grid()
        smoothing_1 = plt.scatter(
            trajectory_plot[:, 0],
            trajectory_plot[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        # smoothing_2
        plt.scatter(
            cones_left_plot[:, 0],
            cones_left_plot[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        # smoothing_3
        plt.scatter(
            cones_right_plot[:, 0],
            cones_right_plot[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        smoothing_4 = plt.plot(
            trajectory_interp_plot[:, 0],
            trajectory_interp_plot[:, 1],
            color=normalize_color((0, 0, 0)),
            linewidth=0.3,
            linestyle="dashed",
            marker="o",
            markersize=2,
            markerfacecolor=normalize_color((0, 0, 0)),
        )
        # smoothing_5
        plt.plot(
            bound_left_interp_plot[:, 0],
            bound_left_interp_plot[:, 1],
            color=normalize_color((0, 0, 0)),
            linewidth=0.3,
            linestyle="dashed",
            marker="o",
            markersize=2,
            markerfacecolor=normalize_color((0, 0, 0)),
        )
        # smoothing_6
        plt.plot(
            bound_right_interp_plot[:, 0],
            bound_right_interp_plot[:, 1],
            color=normalize_color((0, 0, 0)),
            linewidth=0.3,
            linestyle="dashed",
            marker="o",
            markersize=2,
            markerfacecolor=normalize_color((0, 0, 0)),
        )
        smoothing_7 = plt.plot(
            trajectory_smoothed_plot[:, 0],
            trajectory_smoothed_plot[:, 1],
            color=normalize_color((0, 255, 0)),
            linewidth=1.5,
            linestyle="solid",
        )
        smoothing_8 = plt.plot(
            bound_left_smoothed_plot[:, 0],
            bound_left_smoothed_plot[:, 1],
            color=normalize_color((0, 0, 255)),
            linewidth=1.5,
            linestyle="solid",
        )
        smoothing_9 = plt.plot(
            bound_right_smoothed_plot[:, 0],
            bound_right_smoothed_plot[:, 1],
            color=normalize_color((230, 240, 0)),
            linewidth=1.5,
            linestyle="solid",
        )
        smoothing_10 = plt.arrow(
            point1_arrow[0],
            point1_arrow[1],
            vec_arrow[0],
            vec_arrow[1],
            width=0.3,
            head_width=1.0,
            head_length=1.0,
            fc="g",
            ec="g",
        )
        plt.legend(
            [
                smoothing_1,
                smoothing_4[0],
                smoothing_7[0],
                smoothing_8[0],
                smoothing_9[0],
                smoothing_10,
            ],
            [
                "Initial data",
                "Interpolated data",
                "Smoothed trajectory",
                "Smoothed left boundary",
                "Smoothed right boundary",
                "Driving direction",
            ],
            handler_map={
                mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)
            },
        )
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.xlabel("x-distance from original car position in m")
        plt.ylabel("y-distance from original car position in m")

        plt.show()

        # save_plot_to_folder(plt, folder_path, "BSpline Smoothing")
        # plt.close()

        # ------------------------------------------------------------------------------------------------------------------
        # Boundary distance estimation -------------------------------------------------------------------------------------
        # - Plotting the reference trajectory (bound_dists_1)
        # - Plotting the left cones (bound_dists_2)
        # - Plotting the right cones (bound_dists_3)
        # - Plotting the smoothed trajectory (bound_dists_4)
        # - Plotting the smoothed left boundary (bound_dists_5)
        # - Plotting the smoothed right boundary (bound_dists_6)
        # - Plotting the new left boundary (bound_dists_7)
        # - Plotting the new right boundary (bound_dists_8)
        # - Plotting the normals to the smoothed trajectory (bound_dists_9)
        # - Plotting the driving direction (bound_dists_10)
        # ------------------------------------------------------------------------------------------------------------------
        plt.figure("Boundary distance estimation")
        plt.title("Boundary distance estimation")
        plt.grid()
        bound_dists_1 = plt.scatter(
            trajectory_plot[:, 0],
            trajectory_plot[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        # bound_dists_2
        plt.scatter(
            cones_left_plot[:, 0],
            cones_left_plot[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        # bound_dists_3
        plt.scatter(
            cones_right_plot[:, 0],
            cones_right_plot[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        bound_dists_4 = plt.plot(
            trajectory_smoothed_plot[:, 0],
            trajectory_smoothed_plot[:, 1],
            color=normalize_color((0, 255, 0)),
            linewidth=1.0,
            linestyle="dashed",
            marker="o",
            markersize=3,
            markerfacecolor=normalize_color((0, 255, 0)),
        )
        bound_dists_5 = plt.plot(
            bound_left_smoothed_plot[:, 0],
            bound_left_smoothed_plot[:, 1],
            color=normalize_color((0, 0, 0)),
            linewidth=0.5,
            linestyle="dashed",
        )
        # bound_dists_6
        plt.plot(
            bound_right_smoothed_plot[:, 0],
            bound_right_smoothed_plot[:, 1],
            color=normalize_color((0, 0, 0)),
            linewidth=0.5,
            linestyle="dashed",
        )
        bound_dists_7 = plt.plot(
            new_bound_left_plot[:, 0],
            new_bound_left_plot[:, 1],
            color=normalize_color((0, 0, 255)),
            linewidth=1.0,
            linestyle="solid",
            marker="o",
            markersize=3,
            markerfacecolor=normalize_color((0, 0, 255)),
        )
        bound_dists_8 = plt.plot(
            new_bound_right_plot[:, 0],
            new_bound_right_plot[:, 1],
            color=normalize_color((230, 240, 0)),
            linewidth=1.0,
            linestyle="solid",
            marker="o",
            markersize=3,
            markerfacecolor=normalize_color((230, 240, 0)),
        )

        for i in range(new_bound_left_plot.shape[0]):
            temp = np.vstack((new_bound_left_plot[i], new_bound_right_plot[i]))
            bound_dists_9 = plt.plot(
                temp[:, 0],
                temp[:, 1],
                color=normalize_color((255, 0, 0)),
                linewidth=0.7,
                linestyle="solid",
            )

        bound_dists_10 = plt.arrow(
            point1_arrow[0],
            point1_arrow[1],
            vec_arrow[0],
            vec_arrow[1],
            width=0.3,
            head_width=1.0,
            head_length=1.0,
            fc="g",
            ec="g",
        )
        plt.legend(
            [
                bound_dists_1,
                bound_dists_4[0],
                bound_dists_5[0],
                bound_dists_7[0],
                bound_dists_8[0],
                bound_dists_9[0],
                bound_dists_10,
            ],
            [
                "Initial data",
                "Smoothed trajectory",
                "Smoothed boundaries",
                "New left boundary",
                "New right boundary",
                "Trajectory normals",
                "Driving direction",
            ],
            handler_map={
                mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)
            },
        )
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.xlabel("x-distance from original car position in m")
        plt.ylabel("y-distance from original car position in m")

        plt.show()

        # save_plot_to_folder(plt, folder_path, "Boundary Distance Estimation")
        # plt.close()

        # ------------------------------------------------------------------------------------------------------------------
        # Preprocessed track -----------------------------------------------------------------------------------------------
        # - Plotting the preprocessed trajectory (prepped_track_1)
        # - Plotting the left preprocessed boundary (prepped_track_2)
        # - Plotting the right preprocessed boundary (prepped_track_3)
        # - Plotting the driving direction (prepped_track_4)
        # ------------------------------------------------------------------------------------------------------------------
        plt.figure("Preprocessed track")
        plt.title("Preprocessed track")
        plt.grid()
        prepped_track_1 = plt.plot(
            trajectory_smoothed_plot[:, 0],
            trajectory_smoothed_plot[:, 1],
            color=normalize_color((0, 255, 0)),
            linewidth=2.0,
            linestyle="dashed",
        )
        prepped_track_2 = plt.plot(
            new_bound_left_plot[:, 0],
            new_bound_left_plot[:, 1],
            color=normalize_color((0, 0, 255)),
            linewidth=2.0,
            linestyle="solid",
        )
        prepped_track_3 = plt.plot(
            new_bound_right_plot[:, 0],
            new_bound_right_plot[:, 1],
            color=normalize_color((230, 240, 0)),
            linewidth=2.0,
            linestyle="solid",
        )
        prepped_track_4 = plt.arrow(
            point1_arrow[0],
            point1_arrow[1],
            vec_arrow[0],
            vec_arrow[1],
            width=0.3,
            head_width=1.0,
            head_length=1.0,
            fc="g",
            ec="g",
        )
        plt.legend(
            [
                prepped_track_1[0],
                prepped_track_2[0],
                prepped_track_3[0],
                prepped_track_4,
            ],
            [
                "Preprocessed trajectory",
                "Preprocessed left boundary",
                "Preprocessed right boundary",
                "Driving direction",
            ],
            handler_map={
                mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)
            },
        )
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.xlabel("x-distance from original car position in m")
        plt.ylabel("y-distance from original car position in m")

        plt.show()

        # save_plot_to_folder(plt, folder_path, "Preprocessed Track")
        # plt.close()

        # ------------------------------------------------------------------------------------------------------------------
        # Curvature preprocessed track 2D ----------------------------------------------------------------------------------
        # - Plotting the curvature of the preprocessed trajectory (curvature2D_1)
        # ------------------------------------------------------------------------------------------------------------------
        plt.figure("Curvature preprocessed track 2D")
        plt.title("Curvature preprocessed track 2D")
        plt.grid()
        # curvature2D_1
        plt.plot(
            distances_along_traj_plot,
            kappa_prepped_track_plot,
            color=normalize_color((255, 0, 0)),
            linewidth=1.0,
            linestyle="solid",
            label="Curvature preprocessed trajectory",
        )
        plt.legend()
        plt.xlabel("distance from original car position in m")
        plt.ylabel("curvature in rad/m")

        plt.show()

        # save_plot_to_folder(plt, folder_path, "Curvature Preprocessed Track 2D")
        # plt.close()

        # ------------------------------------------------------------------------------------------------------------------
        # Curvature preprocessed track 3D ----------------------------------------------------------------------------------
        # - Plotting the preprocessed trajectory (curvature3D_1)
        # - Plotting the left preprocessed boundary (curvature3D_2)
        # - Plotting the right preprocessed boundary (curvature3D_3)
        # - Plotting the curvature of the preprocessed trajectory (curvature3D_4)
        # ------------------------------------------------------------------------------------------------------------------
        plt.figure("Curvature preprocessed track 3D")
        plt.grid()
        ax_curvature3D = plt.subplot(projection="3d")
        ax_curvature3D.set_title("Curvature preprocessed track 3D")

        # Scale the z-axis such that it does not appear stretched
        curvature3D_scale_x = 1.0
        curvature3D_scale_y = 1.0
        curvature3D_scale_z = 0.3

        # recast get_proj function to use scaling factors for the axes
        ax_curvature3D.get_proj = lambda: np.dot(
            Axes3D.get_proj(ax_curvature3D),
            np.diag(
                [curvature3D_scale_x, curvature3D_scale_y, curvature3D_scale_z, 1.0]
            ),
        )

        curvature3D_1 = ax_curvature3D.plot(
            trajectory_smoothed_plot[:, 0],
            trajectory_smoothed_plot[:, 1],
            color=normalize_color((0, 255, 0)),
            linewidth=1.0,
            linestyle="dashed",
        )
        curvature3D_2 = ax_curvature3D.plot(
            new_bound_left_plot[:, 0],
            new_bound_left_plot[:, 1],
            color=normalize_color((0, 0, 255)),
            linewidth=1.0,
            linestyle="solid",
        )
        curvature3D_3 = ax_curvature3D.plot(
            new_bound_right_plot[:, 0],
            new_bound_right_plot[:, 1],
            color=normalize_color((230, 240, 0)),
            linewidth=1.0,
            linestyle="solid",
        )
        curvature3D_4 = ax_curvature3D.plot(
            trajectory_smoothed_plot[:, 0],
            trajectory_smoothed_plot[:, 1],
            kappa_prepped_track_plot3D,
            color=normalize_color((255, 0, 0)),
            linewidth=1.0,
            linestyle="solid",
        )
        ax_curvature3D.legend(
            [
                curvature3D_1[0],
                curvature3D_2[0],
                curvature3D_3[0],
                curvature3D_4[0],
            ],
            [
                "Preprocessed trajectory",
                "Preprocessed left boundary",
                "Preprocessed right boundary",
                "Curvature preprocessed trajectory",
            ],
        )
        ax_curvature3D.set_xlabel("x-distance from original car position in m")
        ax_curvature3D.set_ylabel("y-distance from original car position in m")
        ax_curvature3D.set_zlabel("curvature in rad/m")
        ax_curvature3D.set_aspect("equalxy")

        plt.show()

        # save_plot_to_folder(plt, folder_path, "Curvature Preprocessed Track 3D")
        # plt.close()

        # ------------------------------------------------------------------------------------------------------------------
        # Curvature derivative preprocessed track 2D ----------------------------------------------------------------------------------
        # - Plotting the curvature of the preprocessed trajectory (Dcurvature2D_1)
        # ------------------------------------------------------------------------------------------------------------------
        plt.figure("Curvature derivative preprocessed track 2D")
        plt.title("Curvature derivative preprocessed track 2D")
        plt.grid()
        # Dcurvature2D_1
        plt.plot(
            distances_along_traj_plot,
            dkappa_prepped_track_plot,
            color=normalize_color((255, 0, 0)),
            linewidth=1.0,
            linestyle="solid",
            label="Curvature derivative preprocessed trajectory",
        )
        plt.legend()
        plt.xlabel("distance from original car postion in m")
        plt.ylabel("curvature derivative in rad/m²")

        plt.show()

        # save_plot_to_folder(plt, folder_path, "Curvature Derivative Preprocessed Track 2D")
        # plt.close()

        # ------------------------------------------------------------------------------------------------------------------
        # Curvature derivative preprocessed track 3D ----------------------------------------------------------------------------------
        # - Plotting the preprocessed trajectory (Dcurvature3D_1)
        # - Plotting the left preprocessed boundary (Dcurvature3D_2)
        # - Plotting the right preprocessed boundary (Dcurvature3D_3)
        # - Plotting the curvature derivative of the preprocessed trajectory (Dcurvature3D_4)
        # ------------------------------------------------------------------------------------------------------------------
        plt.figure("Curvature preprocessed track 3D")
        plt.grid()
        ax_Dcurvature3D = plt.subplot(projection="3d")
        ax_Dcurvature3D.set_title("Curvature derivative preprocessed track 3D")

        # Scale the z-axis such that it does not appear stretched
        Dcurvature3D_scale_x = 1.0
        Dcurvature3D_scale_y = 1.0
        Dcurvature3D_scale_z = 0.3

        # recast get_proj function to use scaling factors for the axes
        ax_Dcurvature3D.get_proj = lambda: np.dot(
            Axes3D.get_proj(ax_Dcurvature3D),
            np.diag(
                [Dcurvature3D_scale_x, Dcurvature3D_scale_y, Dcurvature3D_scale_z, 1.0]
            ),
        )

        Dcurvature3D_1 = ax_Dcurvature3D.plot(
            trajectory_smoothed_plot[:, 0],
            trajectory_smoothed_plot[:, 1],
            color=normalize_color((0, 255, 0)),
            linewidth=1.0,
            linestyle="dashed",
        )
        Dcurvature3D_2 = ax_Dcurvature3D.plot(
            new_bound_left_plot[:, 0],
            new_bound_left_plot[:, 1],
            color=normalize_color((0, 0, 255)),
            linewidth=1.0,
            linestyle="solid",
        )
        Dcurvature3D_3 = ax_Dcurvature3D.plot(
            new_bound_right_plot[:, 0],
            new_bound_right_plot[:, 1],
            color=normalize_color((230, 240, 0)),
            linewidth=1.0,
            linestyle="solid",
        )
        Dcurvature3D_4 = ax_Dcurvature3D.plot(
            trajectory_smoothed_plot[:, 0],
            trajectory_smoothed_plot[:, 1],
            dkappa_prepped_track_plot3D,
            color=normalize_color((255, 0, 0)),
            linewidth=1.0,
            linestyle="solid",
        )
        ax_Dcurvature3D.legend(
            [
                Dcurvature3D_1[0],
                Dcurvature3D_2[0],
                Dcurvature3D_3[0],
                Dcurvature3D_4[0],
            ],
            [
                "Preprocessed trajectory",
                "Preprocessed left boundary",
                "Preprocessed right boundary",
                "Curvature derivative preprocessed trajectory",
            ],
        )
        ax_Dcurvature3D.set_xlabel("x-distance from original car postion in m")
        ax_Dcurvature3D.set_ylabel("y-distance from original car postion in m")
        ax_Dcurvature3D.set_zlabel("curvature derivative in rad/m²")
        ax_Dcurvature3D.set_aspect("equalxy")

        plt.show()

        # save_plot_to_folder(plt, folder_path, "Curvature Derivativ Preprocessed Track 3D")
        # plt.close()

    # Return preprocesing results
    return (
        prepped_track,
        bound_left,
        bound_right,
        coeffs_x,
        coeffs_y,
        M,
        normvec_normalized,
        psi_prepped_track,
        kappa_prepped_track,
        dkappa_prepped_track,
    )


def calc_ax_max_motors(
    v_max: float,
    T_motor_max: float,
    P_max: float,
    num_motors: int,
    gear_ratio: float,
    wheel_radius: float,
    car_mass: float,
) -> np.ndarray:
    """
    author:
    Kwinten Mortier

    .. description::
    Calculates the forward acceleration capabilities of the motors based on the torque limitations of the motor and the
    power limitations set by Formula Student rules.

    .. inputs::
    :param v_max:               maximum velocity of the vehicle in m/s.
    :type v_max:                float
    :param T_motor_max:         maximum torque of the motor in Nm.
    :type T_motor_max:          float
    :param P_max:               maximum combined power in W (set by Formula Student rules).
    :type P_max:                float
    :param num_motors:          number of motors.
    :type num_motors:           int
    :param gear_ratio:          gear ratio of the motor.
    :type gear_ratio:           float
    :param wheel_radius:        outer radius of the wheel in m.
    :type wheel_radius:         float
    :param car_mass:            mass of the vehicle in kg.
    :type car_mass:             float

    .. outputs::
    :return ax_max_motors:      maximum forward acceleration capabilities of the motors at speeds from 0 to v_max.
    :rtype ax_max_motors:       np.ndarray

    .. notes::
    Torque will be maximal until a certain speed is reached, after which the power will be maximal.
    Make sure the correct units are used!

    Result is a 2D array with the first column being the speed in m/s and the second column being the maximum acceleration in m/s².
    """

    # Calculate the maximum acceleration of the motors
    ax_max_motors = np.zeros((int(v_max) + 1, 2))

    for i in range(int(v_max) + 1):
        v = i
        omega = v / wheel_radius
        omega_motor = omega * gear_ratio
        P_motor = num_motors * T_motor_max * omega_motor
        if P_motor <= P_max:
            T_motor = T_motor_max
        else:
            T_motor = P_max / (num_motors * omega_motor)

        T = T_motor * gear_ratio
        F = num_motors * T / wheel_radius
        ax_max = F / car_mass
        ax_max_motors[i] = [v, ax_max]

    return ax_max_motors


def calc_ggv(
    v_max: float,
    acc_limit_long: float,
    acc_limit_lat: float,
) -> np.ndarray:
    """
    author:
    Kwinten Mortier

    .. description::
    Estimates the ggv diagram of the vehicle based on the longitudinal and lateral acceleration limits.

    .. inputs::
    :param v_max:               maximum velocity of the vehicle in m/s.
    :type v_max:                float
    :param acc_limit_long:      longitudinal acceleration limit of the vehicle in m/s².
    :type acc_limit_long:       float
    :param acc_limit_lat:       lateral acceleration limit of the vehicle in m/s².
    :type acc_limit_lat:        float

    .. outputs::
    :return ggv:                ggv diagram of the vehicle.
    :rtype ggv:                 np.ndarray

    .. notes::
    Make sure the correct units are used!

    The estimation is based on the grip circle model. The grip circle model assumes that the maximum longitudinal and lateral
    acceleration limits are independent of each other. The grip circle model is an approximation and the actual ggv diagram
    might differ from the one calculated here. The acceleration limits are assumed to be constant over the entire speed range and are
    extracted from tire data.

    For example: The tire data suggests that the tire can handle up to 1.6 G of lateral and longitudinal acceleration. This means that
    the maximum lateral and longitudinal acceleration limits are 1.6 * 9.80 m/s² = 15.69 m/s².

    Result is a 2D array. The first column is the speed in m/s, the second column is the maximum longitudinal acceleration in m/s²,
    the third column is the maximum lateral acceleration in m/s².
    """

    # Calculate the ggv diagram
    ggv = np.zeros((int(v_max) + 1, 3))

    for i in range(int(v_max) + 1):
        v = i
        ggv[i] = [v, acc_limit_long, acc_limit_lat]

    return ggv


# Needs to be optimised but keep the original code for now (functionality needs to be kept, maybe only closed loop?)
def calc_splines(
    path: np.ndarray,
    el_lengths: np.ndarray = None,
    psi_s: float = None,
    psi_e: float = None,
    use_dist_scaling: bool = True,
) -> tuple:
    """
    author:
    Tim Stahl & Alexander Heilmeier

    .. description::
    Solve for curvature continuous cubic splines (spline parameter t) between given points i (splines evaluated at
    t = 0 and t = 1). The splines must be set up separately for x- and y-coordinate.

    Spline equations:
    P_{x,y}(t)   =  a_3 * t³ +  a_2 * t² + a_1 * t + a_0
    P_{x,y}'(t)  = 3a_3 * t² + 2a_2 * t  + a_1
    P_{x,y}''(t) = 6a_3 * t  + 2a_2

    a * {x; y} = {b_x; b_y}

    .. inputs::
    :param path:                x and y coordinates as the basis for the spline construction (closed or unclosed). If
                                path is provided unclosed, headings psi_s and psi_e are required!
    :type path:                 np.ndarray
    :param el_lengths:          distances between path points (closed or unclosed). The input is optional. The distances
                                are required for the scaling of heading and curvature values. They are calculated using
                                euclidian distances if required but not supplied.
    :type el_lengths:           np.ndarray
    :param psi_s:               orientation of the {start, end} point.
    :type psi_s:                float
    :param psi_e:               orientation of the {start, end} point.
    :type psi_e:                float
    :param use_dist_scaling:    bool flag to indicate if heading and curvature scaling should be performed. This should
                                be done if the distances between the points in the path are not equal.
    :type use_dist_scaling:     bool

    .. outputs::
    :return x_coeff:            spline coefficients of the x-component.
    :rtype x_coeff:             np.ndarray
    :return y_coeff:            spline coefficients of the y-component.
    :rtype y_coeff:             np.ndarray
    :return M:                  LES coefficients.
    :rtype M:                   np.ndarray
    :return normvec_normalized: normalized normal vectors [x, y].
    :rtype normvec_normalized:  np.ndarray

    .. notes::
    Outputs are always unclosed!

    path and el_lengths inputs can either be closed or unclosed, but must be consistent! The function detects
    automatically if the path was inserted closed.

    Coefficient matrices have the form a_0i, a_1i * t, a_2i * t^2, a_3i * t^3.
    """

    # check if path is closed
    if np.all(np.isclose(path[0], path[-1])) and psi_s is None:
        closed = True
    else:
        closed = False

    # check inputs
    if not closed and (psi_s is None or psi_e is None):
        raise RuntimeError("Headings must be provided for unclosed spline calculation!")

    if el_lengths is not None and path.shape[0] != el_lengths.size + 1:
        raise RuntimeError(
            "el_lengths input must be one element smaller than path input!"
        )

    # if distances between path coordinates are not provided but required, calculate euclidean distances as el_lengths
    if use_dist_scaling and el_lengths is None:
        el_lengths = np.sqrt(np.sum(np.power(np.diff(path, axis=0), 2), axis=1))
    elif el_lengths is not None:
        el_lengths = np.copy(el_lengths)

    # if closed and use_dist_scaling active append element length in order to obtain overlapping elements for proper
    # scaling of the last element afterwards
    if use_dist_scaling and closed:
        el_lengths = np.append(el_lengths, el_lengths[0])

    # get number of splines
    no_splines = path.shape[0] - 1

    # calculate scaling factors between every pair of splines
    if use_dist_scaling:
        scaling = el_lengths[:-1] / el_lengths[1:]
    else:
        scaling = np.ones(no_splines - 1)

    # ------------------------------------------------------------------------------------------------------------------
    # DEFINE LINEAR EQUATION SYSTEM ------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # M_{x,y} * a_{x,y} = b_{x,y}) with a_{x,y} being the desired spline param
    # *4 because of 4 parameters in cubic spline
    M = np.zeros((no_splines * 4, no_splines * 4))
    b_x = np.zeros((no_splines * 4, 1))
    b_y = np.zeros((no_splines * 4, 1))

    # create template for M array entries
    # row 1: beginning of current spline should be placed on current point (t = 0)
    # row 2: end of current spline should be placed on next point (t = 1)
    # row 3: heading at end of current spline should be equal to heading at beginning of next spline (t = 1 and t = 0)
    # row 4: curvature at end of current spline should be equal to curvature at beginning of next spline (t = 1 and
    #        t = 0)

    # template_M
    # current point               | next point              | bounds
    # a_0i                                                  = {x,y}_i
    # a_0i + a_1i +  a_2i +  a_3i                           = {x,y}_i+1
    # _      a_1i + 2a_2i + 3a_3i      - a_1i+1             = 0
    # _             2a_2i + 6a_3i               - 2a_2i+1   = 0

    template_M = np.array(
        [
            [1, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 0, 0, 0, 0],
            [0, 1, 2, 3, 0, -1, 0, 0],
            [0, 0, 2, 6, 0, 0, -2, 0],
        ]
    )

    for i in range(no_splines):
        j = i * 4

        if i < no_splines - 1:
            M[j : j + 4, j : j + 8] = template_M

            M[j + 2, j + 5] *= scaling[i]
            M[j + 3, j + 6] *= math.pow(scaling[i], 2)

        else:
            # no curvature and heading bounds on last element (handled afterwards)
            M[j : j + 2, j : j + 4] = [[1, 0, 0, 0], [1, 1, 1, 1]]

        b_x[j : j + 2] = [[path[i, 0]], [path[i + 1, 0]]]
        b_y[j : j + 2] = [[path[i, 1]], [path[i + 1, 1]]]

    # ------------------------------------------------------------------------------------------------------------------
    # SET BOUNDARY CONDITIONS FOR LAST AND FIRST POINT -----------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if not closed:
        # if the path is unclosed we want to fix heading at the start and end point of the path (curvature cannot be
        # determined in this case) -> set heading boundary conditions

        # heading start point
        M[-2, 1] = 1  # heading start point (evaluated at t = 0)

        if el_lengths is None:
            el_length_s = 1.0
        else:
            el_length_s = el_lengths[0]

        b_x[-2] = math.cos(psi_s + math.pi / 2) * el_length_s
        b_y[-2] = math.sin(psi_s + math.pi / 2) * el_length_s

        # heading end point
        M[-1, -4:] = [0, 1, 2, 3]  # heading end point (evaluated at t = 1)

        if el_lengths is None:
            el_length_e = 1.0
        else:
            el_length_e = el_lengths[-1]

        b_x[-1] = math.cos(psi_e + math.pi / 2) * el_length_e
        b_y[-1] = math.sin(psi_e + math.pi / 2) * el_length_e

    else:
        # heading boundary condition (for a closed spline)
        M[-2, 1] = scaling[-1]
        M[-2, -3:] = [-1, -2, -3]
        # b_x[-2] = 0
        # b_y[-2] = 0

        # curvature boundary condition (for a closed spline)
        M[-1, 2] = 2 * math.pow(scaling[-1], 2)
        M[-1, -2:] = [-2, -6]
        # b_x[-1] = 0
        # b_y[-1] = 0

    # ------------------------------------------------------------------------------------------------------------------
    # SOLVE ------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    x_les = np.squeeze(
        np.linalg.solve(M, b_x)
    )  # squeeze removes single-dimensional entries
    y_les = np.squeeze(np.linalg.solve(M, b_y))

    # get coefficients of every piece into one row -> reshape
    coeffs_x = np.reshape(x_les, (no_splines, 4))
    coeffs_y = np.reshape(y_les, (no_splines, 4))

    # get normal vector (behind used here instead of ahead for consistency with other functions) (second coefficient of
    # cubic splines is relevant for the heading)
    normvec = np.stack((coeffs_y[:, 1], -coeffs_x[:, 1]), axis=1)

    # normalize normal vectors
    norm_factors = 1.0 / np.sqrt(np.sum(np.power(normvec, 2), axis=1))
    normvec_normalized = np.expand_dims(norm_factors, axis=1) * normvec

    return coeffs_x, coeffs_y, M, normvec_normalized


def opt_min_curv(
    reftrack: np.ndarray,
    normvectors: np.ndarray,
    A: np.ndarray,
    kappa_bound: float,
    w_veh: float,
    print_debug: bool = False,
    plot_debug: bool = False,
    closed: bool = True,
    psi_s: float = None,
    psi_e: float = None,
    fix_s: bool = False,
    fix_e: bool = False,
) -> tuple:
    """
    author:
    Alexander Heilmeier
    Tim Stahl
    Alexander Wischnewski
    Levent Ögretmen

    .. description::
    This function uses a QP solver to minimize the summed curvature of a path by moving the path points along their
    normal vectors within the track width. The function can be used for closed and unclosed tracks. For unclosed tracks
    the heading psi_s and psi_e is enforced on the first and last point of the reftrack. Furthermore, in case of an
    unclosed track, the first and last point of the reftrack are not subject to optimization and stay same.

    Please refer to our paper for further information:
    Heilmeier, Wischnewski, Hermansdorfer, Betz, Lienkamp, Lohmann
    Minimum Curvature Trajectory Planning and Control for an Autonomous Racecar
    DOI: 10.1080/00423114.2019.1631455

    Hint: CVXOPT can be used as a solver instead of quadprog by uncommenting the import and corresponding code section.

    .. inputs::
    :param reftrack:    array containing the reference track, i.e. a reference line and the according track widths to
                        the right and to the left [x, y, w_tr_left, w_tr_right] (unit is meter, must be unclosed!)
    :type reftrack:     np.ndarray
    :param normvectors: normalized normal vectors for every point of the reference track [x_component, y_component]
                        (unit is meter, must be unclosed!)
    :type normvectors:  np.ndarray
    :param A:           linear equation system matrix for splines (applicable for both, x and y direction)
                        -> System matrices have the form a_i, b_i * t, c_i * t^2, d_i * t^3
                        -> see calc_splines.py for further information or to obtain this matrix
    :type A:            np.ndarray
    :param kappa_bound: curvature boundary to consider during optimization.
    :type kappa_bound:  float
    :param w_veh:       vehicle width in m. It is considered during the calculation of the allowed deviations from the
                        reference line.
    :type w_veh:        float
    :param print_debug: bool flag to print debug messages.
    :type print_debug:  bool
    :param plot_debug:  bool flag to plot the curvatures that are calculated based on the original linearization and on
                        a linearization around the solution.
    :type plot_debug:   bool
    :param closed:      bool flag specifying whether a closed or unclosed track should be assumed
    :type closed:       bool
    :param psi_s:       heading to be enforced at the first point for unclosed tracks
    :type psi_s:        float
    :param psi_e:       heading to be enforced at the last point for unclosed tracks
    :type psi_e:        float
    :param fix_s:       determines if start point is fixed to reference line for unclosed tracks
    :type fix_s:        bool
    :param fix_e:       determines if last point is fixed to reference line for unclosed tracks
    :type fix_e:        bool

    .. outputs::
    :return alpha_mincurv:  solution vector of the opt. problem containing the lateral shift in m for every point.
    :rtype alpha_mincurv:   np.ndarray
    :return curv_error_max: maximum curvature error when comparing the curvature calculated on the basis of the
                            linearization around the original refererence track and around the solution.
    :rtype curv_error_max:  float
    """

    # ------------------------------------------------------------------------------------------------------------------
    # PREPARATIONS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    no_points = reftrack.shape[0]

    no_splines = no_points
    if not closed:
        no_splines -= 1

    # check inputs
    if no_points != normvectors.shape[0]:
        raise RuntimeError("Array size of reftrack should be the same as normvectors!")

    if (
        (no_points * 4 != A.shape[0] and closed)
        or (no_splines * 4 != A.shape[0] and not closed)
        or A.shape[0] != A.shape[1]
    ):
        raise RuntimeError("Spline equation system matrix A has wrong dimensions!")

    # create extraction matrix -> only b_i coefficients of the solved linear equation system are needed for gradient
    # information
    A_ex_b = np.zeros((no_points, no_splines * 4), dtype=int)

    for i in range(no_splines):
        A_ex_b[i, i * 4 + 1] = 1  # 1 * b_ix = E_x * x

    # coefficients for end of spline (t = 1)
    if not closed:
        A_ex_b[-1, -4:] = np.array([0, 1, 2, 3])

    # create extraction matrix -> only c_i coefficients of the solved linear equation system are needed for curvature
    # information
    A_ex_c = np.zeros((no_points, no_splines * 4), dtype=int)

    for i in range(no_splines):
        A_ex_c[i, i * 4 + 2] = 2  # 2 * c_ix = D_x * x

    # coefficients for end of spline (t = 1)
    if not closed:
        A_ex_c[-1, -4:] = np.array([0, 0, 2, 6])

    # invert matrix A resulting from the spline setup linear equation system and apply extraction matrix
    A_inv = np.linalg.inv(A)
    T_c = np.matmul(A_ex_c, A_inv)

    # set up M_x and M_y matrices including the gradient information, i.e. bring normal vectors into matrix form
    M_x = np.zeros((no_splines * 4, no_points))
    M_y = np.zeros((no_splines * 4, no_points))

    for i in range(no_splines):
        j = i * 4

        if i < no_points - 1:
            M_x[j, i] = normvectors[i, 0]
            M_x[j + 1, i + 1] = normvectors[i + 1, 0]

            M_y[j, i] = normvectors[i, 1]
            M_y[j + 1, i + 1] = normvectors[i + 1, 1]
        else:
            M_x[j, i] = normvectors[i, 0]
            M_x[j + 1, 0] = normvectors[0, 0]  # close spline

            M_y[j, i] = normvectors[i, 1]
            M_y[j + 1, 0] = normvectors[0, 1]

    # set up q_x and q_y matrices including the point coordinate information
    q_x = np.zeros((no_splines * 4, 1))
    q_y = np.zeros((no_splines * 4, 1))

    for i in range(no_splines):
        j = i * 4

        if i < no_points - 1:
            q_x[j, 0] = reftrack[i, 0]
            q_x[j + 1, 0] = reftrack[i + 1, 0]

            q_y[j, 0] = reftrack[i, 1]
            q_y[j + 1, 0] = reftrack[i + 1, 1]
        else:
            q_x[j, 0] = reftrack[i, 0]
            q_x[j + 1, 0] = reftrack[0, 0]

            q_y[j, 0] = reftrack[i, 1]
            q_y[j + 1, 0] = reftrack[0, 1]

    # for unclosed tracks, specify start- and end-heading constraints
    if not closed:
        q_x[-2, 0] = math.cos(psi_s + math.pi / 2)
        q_y[-2, 0] = math.sin(psi_s + math.pi / 2)

        q_x[-1, 0] = math.cos(psi_e + math.pi / 2)
        q_y[-1, 0] = math.sin(psi_e + math.pi / 2)

    # set up P_xx, P_xy, P_yy matrices
    x_prime = np.eye(no_points, no_points) * np.matmul(np.matmul(A_ex_b, A_inv), q_x)
    y_prime = np.eye(no_points, no_points) * np.matmul(np.matmul(A_ex_b, A_inv), q_y)

    x_prime_sq = np.power(x_prime, 2)
    y_prime_sq = np.power(y_prime, 2)
    x_prime_y_prime = -2 * np.matmul(x_prime, y_prime)

    curv_den = np.power(x_prime_sq + y_prime_sq, 1.5)  # calculate curvature denominator
    curv_part = np.divide(
        1, curv_den, out=np.zeros_like(curv_den), where=curv_den != 0
    )  # divide where not zero (diag elements)
    curv_part_sq = np.power(curv_part, 2)

    P_xx = np.matmul(curv_part_sq, y_prime_sq)
    P_yy = np.matmul(curv_part_sq, x_prime_sq)
    P_xy = np.matmul(curv_part_sq, x_prime_y_prime)

    # ------------------------------------------------------------------------------------------------------------------
    # SET UP FINAL MATRICES FOR SOLVER ---------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    T_nx = np.matmul(T_c, M_x)
    T_ny = np.matmul(T_c, M_y)

    H_x = np.matmul(T_nx.T, np.matmul(P_xx, T_nx))
    H_xy = np.matmul(T_ny.T, np.matmul(P_xy, T_nx))
    H_y = np.matmul(T_ny.T, np.matmul(P_yy, T_ny))
    H = H_x + H_xy + H_y
    H = (H + H.T) / 2  # make H symmetric

    f_x = 2 * np.matmul(np.matmul(q_x.T, T_c.T), np.matmul(P_xx, T_nx))
    f_xy = np.matmul(np.matmul(q_x.T, T_c.T), np.matmul(P_xy, T_ny)) + np.matmul(
        np.matmul(q_y.T, T_c.T), np.matmul(P_xy, T_nx)
    )
    f_y = 2 * np.matmul(np.matmul(q_y.T, T_c.T), np.matmul(P_yy, T_ny))
    f = f_x + f_xy + f_y
    f = np.squeeze(f)  # remove non-singleton dimensions

    # ------------------------------------------------------------------------------------------------------------------
    # KAPPA CONSTRAINTS ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    Q_x = np.matmul(curv_part, y_prime)
    Q_y = np.matmul(curv_part, x_prime)

    # this part is multiplied by alpha within the optimization (variable part)
    E_kappa = np.matmul(Q_y, T_ny) - np.matmul(Q_x, T_nx)

    # original curvature part (static part)
    k_kappa_ref = np.matmul(Q_y, np.matmul(T_c, q_y)) - np.matmul(
        Q_x, np.matmul(T_c, q_x)
    )

    con_ge = np.ones((no_points, 1)) * kappa_bound - k_kappa_ref
    con_le = -(
        np.ones((no_points, 1)) * -kappa_bound - k_kappa_ref
    )  # multiplied by -1 as only LE conditions are poss.
    con_stack = np.append(con_ge, con_le)

    # ------------------------------------------------------------------------------------------------------------------
    # CALL QUADRATIC PROGRAMMING ALGORITHM -----------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    """
    quadprog interface description taken from
    https://github.com/stephane-caron/qpsolvers/blob/master/qpsolvers/quadprog_.py

    Solve a Quadratic Program defined as:

        minimize
            (1/2) * alpha.T * H * alpha + f.T * alpha

        subject to
            G * alpha <= h
            A * alpha == b

    using quadprog <https://pypi.python.org/pypi/quadprog/>.

    Parameters
    ----------
    H : numpy.array
        Symmetric quadratic-cost matrix.
    f : numpy.array
        Quadratic-cost vector.
    G : numpy.array
        Linear inequality constraint matrix.
    h : numpy.array
        Linear inequality constraint vector.
    A : numpy.array, optional
        Linear equality constraint matrix.
    b : numpy.array, optional
        Linear equality constraint vector.
    initvals : numpy.array, optional
        Warm-start guess vector (not used).

    Returns
    -------
    alpha : numpy.array
            Solution to the QP, if found, otherwise ``None``.

    Note
    ----
    The quadprog solver only considers the lower entries of `H`, therefore it
    will use a wrong cost function if a non-symmetric matrix is provided.
    """

    # calculate allowed deviation from refline
    dev_max_left = reftrack[:, 2] - w_veh / 2
    dev_max_right = reftrack[:, 3] - w_veh / 2

    # constrain resulting path to reference line at start- and end-point for open tracks
    if not closed and fix_s:
        dev_max_left[0] = 0.05
        dev_max_right[0] = 0.05

    if not closed and fix_e:
        dev_max_left[-1] = 0.05
        dev_max_right[-1] = 0.05

    # check that there is space remaining between left and right maximum deviation (both can be negative as well!)
    if np.any(-dev_max_right > dev_max_left) or np.any(-dev_max_left > dev_max_right):
        raise RuntimeError(
            "Problem not solvable, track might be too small to run with current safety distance!"
        )

    # consider value boundaries (-dev_max_left <= alpha <= dev_max_right)
    G = np.vstack((np.eye(no_points), -np.eye(no_points), E_kappa, -E_kappa))
    h = np.append(dev_max_right, dev_max_left)
    h = np.append(h, con_stack)

    # save start time
    t_start = time.perf_counter()

    # solve problem (CVXOPT) -------------------------------------------------------------------------------------------
    # args = [cvxopt.matrix(H), cvxopt.matrix(f), cvxopt.matrix(G), cvxopt.matrix(h)]
    # sol = cvxopt.solvers.qp(*args)
    #
    # if 'optimal' not in sol['status']:
    #     print("WARNING: Optimal solution not found!")
    #
    # alpha_mincurv = np.array(sol['x']).reshape((H.shape[1],))

    # solve problem (quadprog) -----------------------------------------------------------------------------------------
    alpha_mincurv = quadprog.solve_qp(H, -f, -G.T, -h, 0)[0]

    # print runtime into console window
    if print_debug:
        print(
            "Solver runtime opt_min_curv: "
            + "{:.3f}".format(time.perf_counter() - t_start)
            + "s"
        )

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE CURVATURE ERROR ----------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate curvature once based on original linearization and once based on a new linearization around the solution
    q_x_tmp = q_x + np.matmul(M_x, np.expand_dims(alpha_mincurv, 1))
    q_y_tmp = q_y + np.matmul(M_y, np.expand_dims(alpha_mincurv, 1))

    x_prime_tmp = np.eye(no_points, no_points) * np.matmul(
        np.matmul(A_ex_b, A_inv), q_x_tmp
    )
    y_prime_tmp = np.eye(no_points, no_points) * np.matmul(
        np.matmul(A_ex_b, A_inv), q_y_tmp
    )

    x_prime_prime = np.squeeze(
        np.matmul(T_c, q_x) + np.matmul(T_nx, np.expand_dims(alpha_mincurv, 1))
    )
    y_prime_prime = np.squeeze(
        np.matmul(T_c, q_y) + np.matmul(T_ny, np.expand_dims(alpha_mincurv, 1))
    )

    curv_orig_lin = np.zeros(no_points)
    curv_sol_lin = np.zeros(no_points)

    for i in range(no_points):
        curv_orig_lin[i] = (
            x_prime[i, i] * y_prime_prime[i] - y_prime[i, i] * x_prime_prime[i]
        ) / math.pow(math.pow(x_prime[i, i], 2) + math.pow(y_prime[i, i], 2), 1.5)
        curv_sol_lin[i] = (
            x_prime_tmp[i, i] * y_prime_prime[i] - y_prime_tmp[i, i] * x_prime_prime[i]
        ) / math.pow(
            math.pow(x_prime_tmp[i, i], 2) + math.pow(y_prime_tmp[i, i], 2), 1.5
        )

    if plot_debug:
        plt.plot(curv_orig_lin)
        plt.plot(curv_sol_lin)
        plt.legend(("original linearization", "solution based linearization"))
        plt.show()

    # calculate maximum curvature error
    curv_error_max = np.amax(np.abs(curv_sol_lin - curv_orig_lin))

    return alpha_mincurv, curv_error_max


########################################################################################################################
# PLOTTING FUNCTIONS:
# - make_legend_arrow
# - get_plot_name
# - save_plot_to_folder
# - clear_folder


########################################################################################################################
def make_legend_arrow(legend, orig_handle, xdescent, ydescent, width, height, fontsize):
    p = mpatches.FancyArrow(
        0, 0.5 * height, width, 0, length_includes_head=True, head_width=0.75 * height
    )
    return p


def get_plot_name(
    plot_name: str,
) -> str:
    """
    author:
    Kwinten Mortier

    .. description::
    Creates a plot name with a timestamp. The timestamp layout is 'YYYYMMDD_HHMMSS'.

    .. inputs::
    :param plot_name:               name of the plot.
    :type plot_name:                str

    .. outputs::
    :return plot_name_t_stamp:       name of the plot with a timestamp.
    :rtype plot_name_t_stamp:        str
    """

    # Get current date and time
    t_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S").lower()

    # Create plot name with timestamp
    plot_name_t_stamp = f"{plot_name}_{t_stamp}.png"

    return plot_name_t_stamp


def save_plot_to_folder(
    plot: plt.figure,
    folder_path: str,
    plot_filename: str = "plot",
):
    """
    author:
    Kwinten Mortier

    .. description::
    Save a plot to a specified folder.

    .. inputs::
    :param plot:                plot to save.
    :type plot:                 plt.figure.Figure
    :param folder_path:         factor for smoothing the trajectory.
    :type folder_path:          float
    :param plot_filename:       factor for smoothing the trajectory.
    :type plot_filename:        float

    .. outputs::
    None

    .. notes::
    This function saves the plot, no outputs!
    """

    # Create folder if it does not exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Create plot name with timestamp
    plot_filename_t_stamp = get_plot_name(plot_filename)

    # Save plot to folder
    plot.savefig(os.path.join(folder_path, plot_filename_t_stamp), dpi=1200)


def clear_folder(folder_path: str):
    """
    author:
    Kwinten Mortier

    .. description::
    Clear the contents of a folder.

    .. inputs::
    :param folder_path:         path to the folder to clear.
    :type folder_path:          str

    .. outputs::
    None

    .. notes::
    This function clears the contents of the folder, no outputs!
    """
    # Check if folder exists
    if os.path.exists(folder_path):
        # Iterate over the files in the folder
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)
            # Check if the path is a file
            if os.path.isfile(file_path):
                # Remove the file
                os.remove(file_path)
    else:
        rospy.loginfo(f"Folder '{folder_path}' does not exist.")


def iqp_handler(
    reftrack: np.ndarray,
    normvectors: np.ndarray,
    A: np.ndarray,
    spline_len: np.ndarray,
    psi: np.ndarray,  # Not necessary, useful for data
    kappa: np.ndarray,  # Not necessary, useful for data
    dkappa: np.ndarray,  # Not necessary, useful for data
    kappa_bound: float,
    w_veh: float,
    stepsize_interp: float,
    iters_min: int = 3,
    curv_error_allowed: float = 0.01,
    print_debug: bool = False,
    plot_debug: bool = False,
) -> tuple:
    """
    author:
    Alexander Heilmeier
    Marvin Ochsenius

    .. description::
    This function handles the iterative call of the quadratic optimization problem (minimum curvature) during
    trajectory optimization. The interface to this function was kept as similar as possible to the interface of
    opt_min_curv.py.

    The basic idea is to repeatedly call the minimum curvature optimization while we limit restrict the solution space
    for an improved validity (the linearization for the optimization problems is the problem here). After every step
    we update the reference track on the basis of the solution for the next iteration to increase the validity of the
    linearization. Since the optimization problem is based on the assumption of equal stepsizes we have to interpolate
    the track in every iteration.

    Please refer to our paper for further information:
    Heilmeier, Wischnewski, Hermansdorfer, Betz, Lienkamp, Lohmann
    Minimum Curvature Trajectory Planning and Control for an Autonomous Racecar
    DOI: 10.1080/00423114.2019.1631455

    .. inputs::
    :param reftrack:            array containing the reference track, i.e. a reference line and the according track
                                widths to the right and to the left [x, y, w_tr_left, w_tr_right] (unit is meter, must
                                be unclosed!)
    :type reftrack:             np.ndarray
    :param normvectors:         normalized normal vectors for every point of the reference track [x, y]
                                (unit is meter, must be unclosed!)
    :type normvectors:          np.ndarray
    :param A:                   linear equation system matrix for splines (applicable for both, x and y direction)
                                -> System matrices have the form a_i, b_i * t, c_i * t^2, d_i * t^3
                                -> see calc_splines.py for further information or to obtain this matrix
    :type A:                    np.ndarray
    :param spline_len:          spline lengths for every point of the reference track [x, y]
                                (unit is meter, must be unclosed!)
    :type spline_len:           np.ndarray
    :param psi:                 heading for every point of the reference track [x, y]
                                (unit is rad, must be unclosed!)
    :type psi:                  np.ndarray
    :param kappa:               curvature for every point of the reference track [x, y]
                                (unit is 1/m, must be unclosed!)
    :type kappa:                np.ndarray
    :param dkappa:              derivative of curvature for every point of the reference track [x, y]
                                (unit is 1/m^2, must be unclosed!)
    :type dkappa:               np.ndarray
    :param kappa_bound:         curvature boundary to consider during optimization.
    :type kappa_bound:          float
    :param w_veh:               vehicle width in m. It is considered during the calculation of the allowed deviations
                                from the reference line.
    :type w_veh:                float
    :param print_debug:         bool flag to print debug messages.
    :type print_debug:          bool
    :param plot_debug:          bool flag to plot the curvatures that are calculated based on the original linearization
                                and on a linearization around the solution.
    :type plot_debug:           bool
    :param stepsize_interp:     stepsize in meters which is used for an interpolation after the spline approximation.
                                This stepsize determines the steps within the optimization problem.
    :type stepsize_interp:      float
    :param iters_min:           number if minimum iterations of the IQP (termination criterion).
    :type iters_min:            int
    :param curv_error_allowed:  allowed curvature error in rad/m between the original linearization and the
                                linearization around the solution (termination criterion).
    :type curv_error_allowed:   float

    .. outputs::
    :return alpha_mincurv_tmp:  solution vector of the optimization problem containing the lateral shift in m for every
                                point.
    :rtype alpha_mincurv_tmp:   np.ndarray
    :return reftrack_tmp:       reference track data [x, y, w_tr_left, w_tr_right] as it was used in the final iteration
                                of the IQP.
    :rtype reftrack_tmp:        np.ndarray
    :return normvectors_tmp:    normalized normal vectors as they were used in the final iteration of the IQP [x, y].
    :rtype normvectors_tmp:     np.ndarray
    :return spline_len_tmp:     spline lengths of reference track data [x, y, w_tr_left, w_tr_right] as it was used in
                                the final iteration of the IQP.
    :rtype spline_len_tmp:      np.ndarray
    :return psi_reftrack_tmp:   heading of reference track data [x, y, w_tr_left, w_tr_right] as it was used in the
                                final iteration of the IQP.
    :rtype psi_reftrack_tmp:    np.ndarray
    :return kappa_reftrack_tmp: curvtaure of reference track data [x, y, w_tr_left, w_tr_right] as it was used in the
                                final iteration of the IQP.
    :rtype psi_reftrack_tmp:    np.ndarray
    :return dkappa_reftrack_tmp:derivative of curvature of reference track data [x, y, w_tr_left, w_tr_right] as it was
                                used in the final iteration of the IQP.
    :rtype psi_reftrack_tmp:    np.ndarray
    """

    # ------------------------------------------------------------------------------------------------------------------
    # IQP (ITERATIVE QUADRATIC PROGRAMMING) ----------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # set initial data
    reftrack_tmp = reftrack
    normvectors_tmp = normvectors
    A_tmp = A
    spline_len_tmp = spline_len
    psi_reftrack_tmp = psi
    kappa_reftrack_tmp = kappa
    dkappa_reftrack_tmp = dkappa

    rospy.logerr("Starting IQP")

    # loop
    iter_cur = 0

    while True:
        iter_cur += 1

        rospy.logerr("Iteration %d started", iter_cur)
        start = time.perf_counter()

        # calculate intermediate solution and catch sum of squared curvature errors
        alpha_mincurv_tmp, curv_error_max_tmp = opt_min_curv(
            reftrack=reftrack_tmp,
            normvectors=normvectors_tmp,
            A=A_tmp,
            kappa_bound=kappa_bound,
            w_veh=w_veh,
            print_debug=print_debug,
            plot_debug=plot_debug,
        )

        # print some progress information
        if print_debug:
            print(
                "Minimum curvature IQP: iteration %i, curv_error_max: %.4frad/m"
                % (iter_cur, curv_error_max_tmp)
            )

        # restrict solution space to improve validity of the linearization during the first steps
        if iter_cur < iters_min:
            alpha_mincurv_tmp *= iter_cur * 1.0 / iters_min

        # check termination criterions: minimum number of iterations and curvature error
        if (
            iter_cur >= iters_min and curv_error_max_tmp <= curv_error_allowed
        ) or iter_cur == 4:
            end = time.perf_counter()
            rospy.logerr("Iteration %d finished in %f", iter_cur, end - start)
            if print_debug:
                print("Finished IQP!")
            break

        # --------------------------------------------------------------------------------------------------------------
        # INTERPOLATION FOR EQUAL STEPSIZES ----------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        (
            refline_tmp,
            _,
            _,
            _,
            spline_inds_tmp,
            t_values_tmp,
        ) = create_raceline(
            refline=reftrack_tmp[:, :2],
            normvectors=normvectors_tmp,
            alpha=alpha_mincurv_tmp,
            stepsize_interp=stepsize_interp,
        )[:6]

        # calculate new track boundaries on the basis of the intermediate alpha values and interpolate them accordingly
        reftrack_tmp[:, 2] += alpha_mincurv_tmp
        reftrack_tmp[:, 3] -= alpha_mincurv_tmp

        ws_track_tmp = interp_track_widths(
            w_track=reftrack_tmp[:, 2:],
            spline_inds=spline_inds_tmp,
            t_values=t_values_tmp,
            incl_last_point=False,
        )

        # create new reftrack
        reftrack_tmp = np.column_stack((refline_tmp, ws_track_tmp))

        # --------------------------------------------------------------------------------------------------------------
        # CALCULATE NEW SPLINES ON THE BASIS OF THE INTERPOLATED REFERENCE TRACK ---------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # calculate new splines
        refline_tmp_cl = np.vstack((reftrack_tmp[:, :2], reftrack_tmp[0, :2]))

        (
            coeffs_x_tmp,
            coeffs_y_tmp,
            A_tmp,
            normvectors_tmp,
        ) = calc_splines(path=refline_tmp_cl, use_dist_scaling=False)

        # calculate spline lengths
        spline_len_tmp = calc_spline_lengths(
            coeffs_x=coeffs_x_tmp, coeffs_y=coeffs_y_tmp
        )

        # calculate heading, curvature, and first derivative of curvature (analytically)
        (
            psi_reftrack_tmp,
            kappa_reftrack_tmp,
            dkappa_reftrack_tmp,
        ) = calc_head_curv_an(
            coeffs_x=coeffs_x_tmp,
            coeffs_y=coeffs_y_tmp,
            ind_spls=np.arange(reftrack_tmp.shape[0]),
            t_spls=np.zeros(reftrack_tmp.shape[0]),
            calc_dcurv=True,
        )

        end = time.perf_counter()
        rospy.logerr("Iteration %d finished in %f", iter_cur, end - start)

    return (
        alpha_mincurv_tmp,
        reftrack_tmp,
        normvectors_tmp,
        spline_len_tmp,
        psi_reftrack_tmp,
        kappa_reftrack_tmp,
        dkappa_reftrack_tmp,
    )


def interp_track_widths(
    w_track: np.ndarray,
    spline_inds: np.ndarray,
    t_values: np.ndarray,
    incl_last_point: bool = False,
) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    The function (linearly) interpolates the track widths in the same steps as the splines were interpolated before.

    Keep attention that the (multiple) interpolation of track widths can lead to unwanted effects, e.g. that peaks
    in the track widths can disappear if the stepsize is too large (kind of an aliasing effect).

    .. inputs::
    :param w_track:         array containing the track widths in meters [w_track_left, w_track_right] to interpolate,
                            optionally with banking angle in rad: [w_track_left, w_track_right, banking]
    :type w_track:          np.ndarray
    :param spline_inds:     indices that show which spline (and here w_track element) shall be interpolated.
    :type spline_inds:      np.ndarray
    :param t_values:        relative spline coordinate values (t) of every point on the splines specified by spline_inds
    :type t_values:         np.ndarray
    :param incl_last_point: bool flag to show if last point should be included or not.
    :type incl_last_point:  bool

    .. outputs::
    :return w_track_interp: array with interpolated track widths (and optionally banking angle).
    :rtype w_track_interp:  np.ndarray

    .. notes::
    All inputs are unclosed.
    """

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE INTERMEDIATE STEPS -------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    w_track_cl = np.vstack((w_track, w_track[0]))
    no_interp_points = t_values.size  # unclosed

    if incl_last_point:
        w_track_interp = np.zeros((no_interp_points + 1, w_track.shape[1]))
        w_track_interp[-1] = w_track_cl[-1]
    else:
        w_track_interp = np.zeros((no_interp_points, w_track.shape[1]))

    # loop through every interpolation point
    for i in range(no_interp_points):
        # find the spline that hosts the current interpolation point
        ind_spl = spline_inds[i]

        # calculate track widths (linear approximation assumed along one spline)
        w_track_interp[i, 0] = np.interp(
            t_values[i], (0.0, 1.0), w_track_cl[ind_spl : ind_spl + 2, 0]
        )
        w_track_interp[i, 1] = np.interp(
            t_values[i], (0.0, 1.0), w_track_cl[ind_spl : ind_spl + 2, 1]
        )

        if w_track.shape[1] == 3:
            w_track_interp[i, 2] = np.interp(
                t_values[i], (0.0, 1.0), w_track_cl[ind_spl : ind_spl + 2, 2]
            )

    return w_track_interp


def create_raceline(
    refline: np.ndarray,
    normvectors: np.ndarray,
    alpha: np.ndarray,
    stepsize_interp: float,
) -> tuple:
    """
    author:
    Alexander Heilmeier

    .. description::
    This function includes the algorithm part connected to the interpolation of the raceline after the optimization.

    .. inputs::
    :param refline:         array containing the track reference line [x, y] (unit is meter, must be unclosed!)
    :type refline:          np.ndarray
    :param normvectors:     normalized normal vectors for every point of the reference line [x_component, y_component]
                            (unit is meter, must be unclosed!)
    :type normvectors:      np.ndarray
    :param alpha:           solution vector of the optimization problem containing the lateral shift in m for every point.
    :type alpha:            np.ndarray
    :param stepsize_interp: stepsize in meters which is used for the interpolation after the raceline creation.
    :type stepsize_interp:  float

    .. outputs::
    :return raceline_interp:                interpolated raceline [x, y] in m.
    :rtype raceline_interp:                 np.ndarray
    :return A_raceline:                     linear equation system matrix of the splines on the raceline.
    :rtype A_raceline:                      np.ndarray
    :return coeffs_x_raceline:              spline coefficients of the x-component.
    :rtype coeffs_x_raceline:               np.ndarray
    :return coeffs_y_raceline:              spline coefficients of the y-component.
    :rtype coeffs_y_raceline:               np.ndarray
    :return spline_inds_raceline_interp:    contains the indices of the splines that hold the interpolated points.
    :rtype spline_inds_raceline_interp:     np.ndarray
    :return t_values_raceline_interp:       containts the relative spline coordinate values (t) of every point on the
                                            splines.
    :rtype t_values_raceline_interp:        np.ndarray
    :return s_raceline_interp:              total distance in m (i.e. s coordinate) up to every interpolation point.
    :rtype s_raceline_interp:               np.ndarray
    :return spline_lengths_raceline:        lengths of the splines on the raceline in m.
    :rtype spline_lengths_raceline:         np.ndarray
    :return el_lengths_raceline_interp_cl:  distance between every two points on interpolated raceline in m (closed!).
    :rtype el_lengths_raceline_interp_cl:   np.ndarray
    """

    # calculate raceline on the basis of the optimized alpha values
    raceline = refline + np.expand_dims(alpha, 1) * normvectors

    # calculate new splines on the basis of the raceline
    raceline_cl = np.vstack((raceline, raceline[0]))

    (
        coeffs_x_raceline,
        coeffs_y_raceline,
        A_raceline,
        normvectors_raceline,
    ) = calc_splines(path=raceline_cl, use_dist_scaling=False)

    # calculate new spline lengths
    spline_lengths_raceline = calc_spline_lengths(
        coeffs_x=coeffs_x_raceline, coeffs_y=coeffs_y_raceline
    )

    # interpolate splines for evenly spaced raceline points
    (
        raceline_interp,
        spline_inds_raceline_interp,
        t_values_raceline_interp,
        s_raceline_interp,
    ) = interp_splines(
        spline_lengths=spline_lengths_raceline,
        coeffs_x=coeffs_x_raceline,
        coeffs_y=coeffs_y_raceline,
        incl_last_point=False,
        stepsize_approx=stepsize_interp,
    )

    # calculate element lengths
    s_tot_raceline = float(np.sum(spline_lengths_raceline))
    el_lengths_raceline_interp = np.diff(s_raceline_interp)
    el_lengths_raceline_interp_cl = np.append(
        el_lengths_raceline_interp, s_tot_raceline - s_raceline_interp[-1]
    )

    return (
        raceline_interp,
        A_raceline,
        coeffs_x_raceline,
        coeffs_y_raceline,
        spline_inds_raceline_interp,
        t_values_raceline_interp,
        s_raceline_interp,
        spline_lengths_raceline,
        el_lengths_raceline_interp_cl,
    )


def calc_spline_lengths(
    coeffs_x: np.ndarray,
    coeffs_y: np.ndarray,
    quickndirty: bool = False,
    no_interp_points: int = 15,
) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Calculate spline lengths for third order splines defining x- and y-coordinates by usage of intermediate steps.

    .. inputs::
    :param coeffs_x:            coefficient matrix of the x splines with size (no_splines x 4).
    :type coeffs_x:             np.ndarray
    :param coeffs_y:            coefficient matrix of the y splines with size (no_splines x 4).
    :type coeffs_y:             np.ndarray
    :param quickndirty:         True returns lengths based on distance between first and last spline point instead of
                                using interpolation.
    :type quickndirty:          bool
    :param no_interp_points:    length calculation is carried out with the given number of interpolation steps.
    :type no_interp_points:     int

    .. outputs::
    :return spline_lengths:     length of every spline segment.
    :rtype spline_lengths:      np.ndarray

    .. notes::
    len(coeffs_x) = len(coeffs_y) = len(spline_lengths)
    """

    # ------------------------------------------------------------------------------------------------------------------
    # PREPARATIONS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # check inputs
    if coeffs_x.shape[0] != coeffs_y.shape[0]:
        raise RuntimeError("Coefficient matrices must have the same length!")

    # catch case with only one spline
    if coeffs_x.size == 4 and coeffs_x.shape[0] == 4:
        coeffs_x = np.expand_dims(coeffs_x, 0)
        coeffs_y = np.expand_dims(coeffs_y, 0)

    # get number of splines and create output array
    no_splines = coeffs_x.shape[0]
    spline_lengths = np.zeros(no_splines)

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE LENGHTS ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if quickndirty:
        for i in range(no_splines):
            spline_lengths[i] = math.sqrt(
                math.pow(np.sum(coeffs_x[i]) - coeffs_x[i, 0], 2)
                + math.pow(np.sum(coeffs_y[i]) - coeffs_y[i, 0], 2)
            )

    else:
        # loop through all the splines and calculate intermediate coordinates
        t_steps = np.linspace(0.0, 1.0, no_interp_points)
        spl_coords = np.zeros((no_interp_points, 2))

        for i in range(no_splines):
            spl_coords[:, 0] = (
                coeffs_x[i, 0]
                + coeffs_x[i, 1] * t_steps
                + coeffs_x[i, 2] * np.power(t_steps, 2)
                + coeffs_x[i, 3] * np.power(t_steps, 3)
            )
            spl_coords[:, 1] = (
                coeffs_y[i, 0]
                + coeffs_y[i, 1] * t_steps
                + coeffs_y[i, 2] * np.power(t_steps, 2)
                + coeffs_y[i, 3] * np.power(t_steps, 3)
            )

            spline_lengths[i] = np.sum(
                np.sqrt(np.sum(np.power(np.diff(spl_coords, axis=0), 2), axis=1))
            )

    return spline_lengths


def interp_splines(
    coeffs_x: np.ndarray,
    coeffs_y: np.ndarray,
    spline_lengths: np.ndarray = None,
    incl_last_point: bool = False,
    stepsize_approx: float = None,
    stepnum_fixed: list = None,
) -> tuple:
    """
    author:
    Alexander Heilmeier & Tim Stahl

    .. description::
    Interpolate points on one or more splines with third order. The last point (i.e. t = 1.0)
    can be included if option is set accordingly (should be prevented for a closed raceline in most cases). The
    algorithm keeps stepsize_approx as good as possible.

    .. inputs::
    :param coeffs_x:        coefficient matrix of the x splines with size (no_splines x 4).
    :type coeffs_x:         np.ndarray
    :param coeffs_y:        coefficient matrix of the y splines with size (no_splines x 4).
    :type coeffs_y:         np.ndarray
    :param spline_lengths:  array containing the lengths of the inserted splines with size (no_splines x 1).
    :type spline_lengths:   np.ndarray
    :param incl_last_point: flag to set if last point should be kept or removed before return.
    :type incl_last_point:  bool
    :param stepsize_approx: desired stepsize of the points after interpolation.                      \\ Provide only one
    :type stepsize_approx:  float
    :param stepnum_fixed:   return a fixed number of coordinates per spline, list of length no_splines. \\ of these two!
    :type stepnum_fixed:    list

    .. outputs::
    :return path_interp:    interpolated path points.
    :rtype path_interp:     np.ndarray
    :return spline_inds:    contains the indices of the splines that hold the interpolated points.
    :rtype spline_inds:     np.ndarray
    :return t_values:       containts the relative spline coordinate values (t) of every point on the splines.
    :rtype t_values:        np.ndarray
    :return dists_interp:   total distance up to every interpolation point.
    :rtype dists_interp:    np.ndarray

    .. notes::
    len(coeffs_x) = len(coeffs_y) = len(spline_lengths)

    len(path_interp = len(spline_inds) = len(t_values) = len(dists_interp)
    """

    # ------------------------------------------------------------------------------------------------------------------
    # INPUT CHECKS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # check sizes
    if coeffs_x.shape[0] != coeffs_y.shape[0]:
        raise RuntimeError("Coefficient matrices must have the same length!")

    if spline_lengths is not None and coeffs_x.shape[0] != spline_lengths.size:
        raise RuntimeError("coeffs_x/y and spline_lengths must have the same length!")

    # check if coeffs_x and coeffs_y have exactly two dimensions and raise error otherwise
    if not (coeffs_x.ndim == 2 and coeffs_y.ndim == 2):
        raise RuntimeError("Coefficient matrices do not have two dimensions!")

    # check if step size specification is valid
    if (stepsize_approx is None and stepnum_fixed is None) or (
        stepsize_approx is not None and stepnum_fixed is not None
    ):
        raise RuntimeError(
            "Provide one of 'stepsize_approx' and 'stepnum_fixed' and set the other to 'None'!"
        )

    if stepnum_fixed is not None and len(stepnum_fixed) != coeffs_x.shape[0]:
        raise RuntimeError(
            "The provided list 'stepnum_fixed' must hold an entry for every spline!"
        )

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE NUMBER OF INTERPOLATION POINTS AND ACCORDING DISTANCES -------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if stepsize_approx is not None:
        # get the total distance up to the end of every spline (i.e. cumulated distances)
        if spline_lengths is None:
            spline_lengths = calc_spline_lengths(
                coeffs_x=coeffs_x, coeffs_y=coeffs_y, quickndirty=False
            )

        dists_cum = np.cumsum(spline_lengths)

        # calculate number of interpolation points and distances (+1 because last point is included at first)
        no_interp_points = math.ceil(dists_cum[-1] / stepsize_approx) + 1
        dists_interp = np.linspace(0.0, dists_cum[-1], no_interp_points)

    else:
        # get total number of points to be sampled (subtract overlapping points)
        no_interp_points = sum(stepnum_fixed) - (len(stepnum_fixed) - 1)
        dists_interp = None

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE INTERMEDIATE STEPS -------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # create arrays to save the values
    path_interp = np.zeros((no_interp_points, 2))  # raceline coords (x, y) array
    spline_inds = np.zeros(
        no_interp_points, dtype=int
    )  # save the spline index to which a point belongs
    t_values = np.zeros(no_interp_points)  # save t values

    if stepsize_approx is not None:
        # --------------------------------------------------------------------------------------------------------------
        # APPROX. EQUAL STEP SIZE ALONG PATH OF ADJACENT SPLINES -------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # loop through all the elements and create steps with stepsize_approx
        for i in range(no_interp_points - 1):
            # find the spline that hosts the current interpolation point
            j = np.argmax(dists_interp[i] < dists_cum)
            spline_inds[i] = j

            # get spline t value depending on the progress within the current element
            if j > 0:
                t_values[i] = (dists_interp[i] - dists_cum[j - 1]) / spline_lengths[j]
            else:
                if spline_lengths.ndim == 0:
                    t_values[i] = dists_interp[i] / spline_lengths
                else:
                    t_values[i] = dists_interp[i] / spline_lengths[0]

            # calculate coords
            path_interp[i, 0] = (
                coeffs_x[j, 0]
                + coeffs_x[j, 1] * t_values[i]
                + coeffs_x[j, 2] * math.pow(t_values[i], 2)
                + coeffs_x[j, 3] * math.pow(t_values[i], 3)
            )

            path_interp[i, 1] = (
                coeffs_y[j, 0]
                + coeffs_y[j, 1] * t_values[i]
                + coeffs_y[j, 2] * math.pow(t_values[i], 2)
                + coeffs_y[j, 3] * math.pow(t_values[i], 3)
            )

    else:
        # --------------------------------------------------------------------------------------------------------------
        # FIXED STEP SIZE FOR EVERY SPLINE SEGMENT ---------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        j = 0

        for i in range(len(stepnum_fixed)):
            # skip last point except for last segment
            if i < len(stepnum_fixed) - 1:
                t_values[j : (j + stepnum_fixed[i] - 1)] = np.linspace(
                    0, 1, stepnum_fixed[i]
                )[:-1]
                spline_inds[j : (j + stepnum_fixed[i] - 1)] = i
                j += stepnum_fixed[i] - 1

            else:
                t_values[j : (j + stepnum_fixed[i])] = np.linspace(
                    0, 1, stepnum_fixed[i]
                )
                spline_inds[j : (j + stepnum_fixed[i])] = i
                j += stepnum_fixed[i]

        t_set = np.column_stack(
            (
                np.ones(no_interp_points),
                t_values,
                np.power(t_values, 2),
                np.power(t_values, 3),
            )
        )

        # remove overlapping samples
        n_samples = np.array(stepnum_fixed)
        n_samples[:-1] -= 1

        path_interp[:, 0] = np.sum(
            np.multiply(np.repeat(coeffs_x, n_samples, axis=0), t_set), axis=1
        )
        path_interp[:, 1] = np.sum(
            np.multiply(np.repeat(coeffs_y, n_samples, axis=0), t_set), axis=1
        )

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE LAST POINT IF REQUIRED (t = 1.0) -----------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if incl_last_point:
        path_interp[-1, 0] = np.sum(coeffs_x[-1])
        path_interp[-1, 1] = np.sum(coeffs_y[-1])
        spline_inds[-1] = coeffs_x.shape[0] - 1
        t_values[-1] = 1.0

    else:
        path_interp = path_interp[:-1]
        spline_inds = spline_inds[:-1]
        t_values = t_values[:-1]

        if dists_interp is not None:
            dists_interp = dists_interp[:-1]

    # NOTE: dists_interp is None, when using a fixed step size
    return path_interp, spline_inds, t_values, dists_interp


def calc_head_curv_an(
    coeffs_x: np.ndarray,
    coeffs_y: np.ndarray,
    ind_spls: np.ndarray,
    t_spls: np.ndarray,
    calc_curv: bool = True,
    calc_dcurv: bool = False,
) -> tuple:
    """
    author:
    Alexander Heilmeier
    Marvin Ochsenius (dcurv extension)

    .. description::
    Analytical calculation of heading psi, curvature kappa, and first derivative of the curvature dkappa
    on the basis of third order splines for x- and y-coordinate.

    .. inputs::
    :param coeffs_x:    coefficient matrix of the x splines with size (no_splines x 4).
    :type coeffs_x:     np.ndarray
    :param coeffs_y:    coefficient matrix of the y splines with size (no_splines x 4).
    :type coeffs_y:     np.ndarray
    :param ind_spls:    contains the indices of the splines that hold the points for which we want to calculate heading/curv.
    :type ind_spls:     np.ndarray
    :param t_spls:      containts the relative spline coordinate values (t) of every point on the splines.
    :type t_spls:       np.ndarray
    :param calc_curv:   bool flag to show if curvature should be calculated as well (kappa is set 0.0 otherwise).
    :type calc_curv:    bool
    :param calc_dcurv:  bool flag to show if first derivative of curvature should be calculated as well.
    :type calc_dcurv:   bool

    .. outputs::
    :return psi:        heading at every point.
    :rtype psi:         float
    :return kappa:      curvature at every point.
    :rtype kappa:       float
    :return dkappa:     first derivative of curvature at every point (if calc_dcurv bool flag is True).
    :rtype dkappa:      float

    .. notes::
    len(ind_spls) = len(t_spls) = len(psi) = len(kappa) = len(dkappa)
    """

    # check inputs
    if coeffs_x.shape[0] != coeffs_y.shape[0]:
        raise ValueError("Coefficient matrices must have the same length!")

    if ind_spls.size != t_spls.size:
        raise ValueError("ind_spls and t_spls must have the same length!")

    if not calc_curv and calc_dcurv:
        raise ValueError("dkappa cannot be calculated without kappa!")

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE HEADING ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate required derivatives
    x_d = (
        coeffs_x[ind_spls, 1]
        + 2 * coeffs_x[ind_spls, 2] * t_spls
        + 3 * coeffs_x[ind_spls, 3] * np.power(t_spls, 2)
    )

    y_d = (
        coeffs_y[ind_spls, 1]
        + 2 * coeffs_y[ind_spls, 2] * t_spls
        + 3 * coeffs_y[ind_spls, 3] * np.power(t_spls, 2)
    )

    # calculate heading psi (pi/2 must be substracted due to our convention that psi = 0 is north)
    psi = np.arctan2(y_d, x_d) - math.pi / 2
    psi = normalize_psi(psi)

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE CURVATURE ----------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if calc_curv:
        # calculate required derivatives
        x_dd = 2 * coeffs_x[ind_spls, 2] + 6 * coeffs_x[ind_spls, 3] * t_spls

        y_dd = 2 * coeffs_y[ind_spls, 2] + 6 * coeffs_y[ind_spls, 3] * t_spls

        # calculate curvature kappa
        kappa = (x_d * y_dd - y_d * x_dd) / np.power(
            np.power(x_d, 2) + np.power(y_d, 2), 1.5
        )

    else:
        kappa = 0.0

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE FIRST DERIVATIVE OF CURVATURE --------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if calc_dcurv:
        # calculate required derivatives
        x_ddd = 6 * coeffs_x[ind_spls, 3]

        y_ddd = 6 * coeffs_y[ind_spls, 3]

        # calculate first derivative of curvature dkappa
        dkappa = (
            (np.power(x_d, 2) + np.power(y_d, 2)) * (x_d * y_ddd - y_d * x_ddd)
            - 3 * (x_d * y_dd - y_d * x_dd) * (x_d * x_dd + y_d * y_dd)
        ) / np.power(np.power(x_d, 2) + np.power(y_d, 2), 3)

        return psi, kappa, dkappa

    else:
        return psi, kappa


def normalize_psi(psi: Union[np.ndarray, float]) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Normalize heading psi such that [-pi,pi[ holds as interval boundaries.

    .. inputs::
    :param psi:         array containing headings psi to be normalized.
    :type psi:          Union[np.ndarray, float]

    .. outputs::
    :return psi_out:    array with normalized headings psi.
    :rtype psi_out:     np.ndarray

    .. notes::
    len(psi) = len(psi_out)
    """

    # use modulo operator to remove multiples of 2*pi
    psi_out = np.sign(psi) * np.mod(np.abs(psi), 2 * math.pi)

    # restrict psi to [-pi,pi[
    if type(psi_out) is np.ndarray:
        psi_out[psi_out >= math.pi] -= 2 * math.pi
        psi_out[psi_out < -math.pi] += 2 * math.pi

    else:
        if psi_out >= math.pi:
            psi_out -= 2 * math.pi
        elif psi_out < -math.pi:
            psi_out += 2 * math.pi

    return psi_out


def check_normals_crossing(
    track: np.ndarray, normvec_normalized: np.ndarray, horizon: int = 10
) -> bool:
    """
    author:
    Alexander Heilmeier

    .. description::
    This function checks spline normals for crossings. Returns True if a crossing was found, otherwise False.

    .. inputs::
    :param track:               array containing the track [x, y, w_tr_right, w_tr_left] to check
    :type track:                np.ndarray
    :param normvec_normalized:  array containing normalized normal vectors for every track point
                                [x_component, y_component]
    :type normvec_normalized:   np.ndarray
    :param horizon:             determines the number of normals in forward and backward direction that are checked
                                against each normal on the line
    :type horizon:              int

    .. outputs::
    :return found_crossing:     bool value indicating if a crossing was found or not
    :rtype found_crossing:      bool

    .. notes::
    The checks can take a while if full check is performed. Inputs are unclosed.
    """

    # check input
    no_points = track.shape[0]

    if horizon >= no_points:
        raise RuntimeError(
            "Horizon of %i points is too large for a track with %i points, reduce horizon!"
            % (horizon, no_points)
        )

    elif horizon >= no_points / 2:
        print(
            "WARNING: Horizon of %i points makes no sense for a track with %i points, reduce horizon!"
            % (horizon, no_points)
        )

    # initialization
    les_mat = np.zeros((2, 2))
    idx_list = list(range(0, no_points))
    idx_list = idx_list[-horizon:] + idx_list + idx_list[:horizon]

    # loop through all points of the track to check for crossings in their neighbourhoods
    for idx in range(no_points):
        # determine indices of points in the neighbourhood of the current index
        idx_neighbours = idx_list[idx : idx + 2 * horizon + 1]
        del idx_neighbours[horizon]
        idx_neighbours = np.array(idx_neighbours)

        # remove indices of normal vectors that are collinear to the current index
        is_collinear_b = np.isclose(
            np.cross(normvec_normalized[idx], normvec_normalized[idx_neighbours]), 0.0
        )
        idx_neighbours_rel = idx_neighbours[np.nonzero(np.invert(is_collinear_b))[0]]

        # check crossings solving an LES
        for idx_comp in list(idx_neighbours_rel):
            # LES: x_1 + lambda_1 * nx_1 = x_2 + lambda_2 * nx_2; y_1 + lambda_1 * ny_1 = y_2 + lambda_2 * ny_2;
            const = track[idx_comp, :2] - track[idx, :2]
            les_mat[:, 0] = normvec_normalized[idx]
            les_mat[:, 1] = -normvec_normalized[idx_comp]

            # solve LES
            lambdas = np.linalg.solve(les_mat, const)

            # we have a crossing within the relevant part if both lambdas lie between -w_tr_left and w_tr_right
            if (
                -track[idx, 3] <= lambdas[0] <= track[idx, 2]
                and -track[idx_comp, 3] <= lambdas[1] <= track[idx_comp, 2]
            ):
                return True  # found crossing

    return False


def side_of_line(
    a: Union[tuple, np.ndarray],
    b: Union[tuple, np.ndarray],
    z: Union[tuple, np.ndarray],
) -> float:
    """
    author:
    Alexander Heilmeier

    .. description::
    Function determines if a point z is on the left or right side of a line from a to b. It is based on the z component
    orientation of the cross product, see question on
    https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line

    .. inputs::
    :param a:       point coordinates [x, y]
    :type a:        Union[tuple, np.ndarray]
    :param b:       point coordinates [x, y]
    :type b:        Union[tuple, np.ndarray]
    :param z:       point coordinates [x, y]
    :type z:        Union[tuple, np.ndarray]

    .. outputs::
    :return side:   0.0 = on line, 1.0 = left side, -1.0 = right side.
    :rtype side:    float
    """

    # calculate side
    side = np.sign((b[0] - a[0]) * (z[1] - a[1]) - (b[1] - a[1]) * (z[0] - a[0]))

    return side


def conv_filt(signal: np.ndarray, filt_window: int, closed: bool) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    modified by:
    Tim Stahl

    .. description::
    Filter a given temporal signal using a convolution (moving average) filter.

    .. inputs::
    :param signal:          temporal signal that should be filtered (always unclosed).
    :type signal:           np.ndarray
    :param filt_window:     filter window size for moving average filter (must be odd).
    :type filt_window:      int
    :param closed:          flag showing if the signal can be considered as closable, e.g. for velocity profiles.
    :type closed:           bool

    .. outputs::
    :return signal_filt:    filtered input signal (always unclosed).
    :rtype signal_filt:     np.ndarray

    .. notes::
    signal input is always unclosed!

    len(signal) = len(signal_filt)
    """

    # check if window width is odd
    if not filt_window % 2 == 1:
        raise RuntimeError("Window width of moving average filter must be odd!")

    # calculate half window width - 1
    w_window_half = int((filt_window - 1) / 2)

    # apply filter
    if closed:
        # temporarily add points in front of and behind signal
        signal_tmp = np.concatenate(
            (signal[-w_window_half:], signal, signal[:w_window_half]), axis=0
        )

        # apply convolution filter used as a moving average filter and remove temporary points
        signal_filt = np.convolve(
            signal_tmp, np.ones(filt_window) / float(filt_window), mode="same"
        )[w_window_half:-w_window_half]

    else:
        # implementation 1: include boundaries during filtering
        # no_points = signal.size
        # signal_filt = np.zeros(no_points)
        #
        # for i in range(no_points):
        #     if i < w_window_half:
        #         signal_filt[i] = np.average(signal[:i + w_window_half + 1])
        #
        #     elif i < no_points - w_window_half:
        #         signal_filt[i] = np.average(signal[i - w_window_half:i + w_window_half + 1])
        #
        #     else:
        #         signal_filt[i] = np.average(signal[i - w_window_half:])

        # implementation 2: start filtering at w_window_half and stop at -w_window_half
        signal_filt = np.copy(signal)
        signal_filt[w_window_half:-w_window_half] = np.convolve(
            signal, np.ones(filt_window) / float(filt_window), mode="same"
        )[w_window_half:-w_window_half]

    return signal_filt


def calc_vel_profile(
    ax_max_machines: np.ndarray,
    kappa: np.ndarray,
    el_lengths: np.ndarray,
    closed: bool,
    drag_coeff: float,
    m_veh: float,
    ggv: np.ndarray = None,
    loc_gg: np.ndarray = None,
    v_max: float = None,
    dyn_model_exp: float = 1.0,
    mu: np.ndarray = None,
    v_start: float = None,
    v_end: float = None,
    filt_window: int = None,
) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    modified by:
    Tim Stahl

    .. description::
    Calculates a velocity profile using the tire and motor limits as good as possible.

    .. inputs::
    :param ax_max_machines: longitudinal acceleration limits by the electrical motors: [vx, ax_max_machines]. Velocity
                            in m/s, accelerations in m/s2. They should be handed in without considering drag resistance,
                            i.e. simply by calculating F_x_drivetrain / m_veh
    :type ax_max_machines:  np.ndarray
    :param kappa:           curvature profile of given trajectory in rad/m (always unclosed).
    :type kappa:            np.ndarray
    :param el_lengths:      element lengths (distances between coordinates) of given trajectory.
    :type el_lengths:       np.ndarray
    :param closed:          flag to set if the velocity profile must be calculated for a closed or unclosed trajectory.
    :type closed:           bool
    :param drag_coeff:      drag coefficient including all constants: drag_coeff = 0.5 * c_w * A_front * rho_air
    :type drag_coeff:       float
    :param m_veh:           vehicle mass in kg.
    :type m_veh:            float
    :param ggv:             ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2.
                            ATTENTION: Insert either ggv + mu (optional) or loc_gg!
    :type ggv:              np.ndarray
    :param loc_gg:          local gg diagrams along the path points: [[ax_max_0, ay_max_0], [ax_max_1, ay_max_1], ...],
                            accelerations in m/s2. ATTENTION: Insert either ggv + mu (optional) or loc_gg!
    :type loc_gg:           np.ndarray
    :param v_max:           Maximum longitudinal speed in m/s (optional if ggv is supplied, taking the minimum of the
                            fastest velocities covered by the ggv and ax_max_machines arrays then).
    :type v_max:            float
    :param dyn_model_exp:   exponent used in the vehicle dynamics model (usual range [1.0,2.0]).
    :type dyn_model_exp:    float
    :param mu:              friction coefficients (always unclosed).
    :type mu:               np.ndarray
    :param v_start:         start velocity in m/s (used in unclosed case only).
    :type v_start:          float
    :param v_end:           end velocity in m/s (used in unclosed case only).
    :type v_end:            float
    :param filt_window:     filter window size for moving average filter (must be odd).
    :type filt_window:      int

    .. outputs::
    :return vx_profile:     calculated velocity profile (always unclosed).
    :rtype vx_profile:      np.ndarray

    .. notes::
    All inputs must be inserted unclosed, i.e. kappa[-1] != kappa[0], even if closed is set True! (el_lengths is kind of
    closed if closed is True of course!)

    case closed is True:
    len(kappa) = len(el_lengths) = len(mu) = len(vx_profile)

    case closed is False:
    len(kappa) = len(el_lengths) + 1 = len(mu) = len(vx_profile)
    """

    # ------------------------------------------------------------------------------------------------------------------
    # INPUT CHECKS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # check if either ggv (and optionally mu) or loc_gg are handed in
    if (ggv is not None or mu is not None) and loc_gg is not None:
        raise RuntimeError(
            "Either ggv and optionally mu OR loc_gg must be supplied, not both (or all) of them!"
        )

    if ggv is None and loc_gg is None:
        raise RuntimeError("Either ggv or loc_gg must be supplied!")

    # check shape of loc_gg
    if loc_gg is not None:
        if loc_gg.ndim != 2:
            raise RuntimeError("loc_gg must have two dimensions!")

        if loc_gg.shape[0] != kappa.size:
            raise RuntimeError("Length of loc_gg and kappa must be equal!")

        if loc_gg.shape[1] != 2:
            raise RuntimeError("loc_gg must consist of two columns: [ax_max, ay_max]!")

    # check shape of ggv
    if ggv is not None and ggv.shape[1] != 3:
        raise RuntimeError(
            "ggv diagram must consist of the three columns [vx, ax_max, ay_max]!"
        )

    # check size of mu
    if mu is not None and kappa.size != mu.size:
        raise RuntimeError("kappa and mu must have the same length!")

    # check size of kappa and element lengths
    if closed and kappa.size != el_lengths.size:
        raise RuntimeError("kappa and el_lengths must have the same length if closed!")

    elif not closed and kappa.size != el_lengths.size + 1:
        raise RuntimeError("kappa must have the length of el_lengths + 1 if unclosed!")

    # check start and end velocities
    if not closed and v_start is None:
        raise RuntimeError("v_start must be provided for the unclosed case!")

    if v_start is not None and v_start < 0.0:
        v_start = 0.0
        print("WARNING: Input v_start was < 0.0. Using v_start = 0.0 instead!")

    if v_end is not None and v_end < 0.0:
        v_end = 0.0
        print("WARNING: Input v_end was < 0.0. Using v_end = 0.0 instead!")

    # check dyn_model_exp
    if not 1.0 <= dyn_model_exp <= 2.0:
        print(
            "WARNING: Exponent for the vehicle dynamics model should be in the range [1.0, 2.0]!"
        )

    # check shape of ax_max_machines
    if ax_max_machines.shape[1] != 2:
        raise RuntimeError(
            "ax_max_machines must consist of the two columns [vx, ax_max_machines]!"
        )

    # check v_max
    if v_max is None:
        if ggv is None:
            raise RuntimeError("v_max must be supplied if ggv is None!")
        else:
            v_max = min(ggv[-1, 0], ax_max_machines[-1, 0])

    else:
        # check if ggv covers velocity until v_max
        if ggv is not None and ggv[-1, 0] < v_max:
            raise RuntimeError(
                "ggv has to cover the entire velocity range of the car (i.e. >= v_max)!"
            )

        # check if ax_max_machines covers velocity until v_max
        if ax_max_machines[-1, 0] < v_max:
            raise RuntimeError(
                "ax_max_machines has to cover the entire velocity range of the car (i.e. >= v_max)!"
            )

    # ------------------------------------------------------------------------------------------------------------------
    # BRINGING GGV OR LOC_GG INTO SHAPE FOR EQUAL HANDLING AFTERWARDS --------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    """For an equal/easier handling of every case afterwards we bring all cases into a form where the local ggv is made
    available for every waypoint, i.e. [ggv_0, ggv_1, ggv_2, ...] -> we have a three dimensional array p_ggv (path_ggv)
    where the first dimension is the waypoint, the second is the velocity and the third is the two acceleration columns
    -> DIM = NO_WAYPOINTS_CLOSED x NO_VELOCITY ENTRIES x 3"""

    # CASE 1: ggv supplied -> copy it for every waypoint
    if ggv is not None:
        p_ggv = np.repeat(np.expand_dims(ggv, axis=0), kappa.size, axis=0)
        op_mode = "ggv"

    # CASE 2: local gg diagram supplied -> add velocity dimension (artificial velocity of 10.0 m/s)
    else:
        p_ggv = np.expand_dims(
            np.column_stack((np.ones(loc_gg.shape[0]) * 10.0, loc_gg)), axis=1
        )
        op_mode = "loc_gg"

    # ------------------------------------------------------------------------------------------------------------------
    # SPEED PROFILE CALCULATION (FB) -----------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # transform curvature kappa into corresponding radii (abs because curvature has a sign in our convention)
    radii = np.abs(
        np.divide(1.0, kappa, out=np.full(kappa.size, np.inf), where=kappa != 0.0)
    )

    # call solver
    if not closed:
        vx_profile = __solver_fb_unclosed(
            p_ggv=p_ggv,
            ax_max_machines=ax_max_machines,
            v_max=v_max,
            radii=radii,
            el_lengths=el_lengths,
            mu=mu,
            v_start=v_start,
            v_end=v_end,
            dyn_model_exp=dyn_model_exp,
            drag_coeff=drag_coeff,
            m_veh=m_veh,
            op_mode=op_mode,
        )

    else:
        vx_profile = __solver_fb_closed(
            p_ggv=p_ggv,
            ax_max_machines=ax_max_machines,
            v_max=v_max,
            radii=radii,
            el_lengths=el_lengths,
            mu=mu,
            dyn_model_exp=dyn_model_exp,
            drag_coeff=drag_coeff,
            m_veh=m_veh,
            op_mode=op_mode,
        )

    # ------------------------------------------------------------------------------------------------------------------
    # POSTPROCESSING ---------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if filt_window is not None:
        vx_profile = conv_filt(
            signal=vx_profile, filt_window=filt_window, closed=closed
        )

    return vx_profile


def __solver_fb_unclosed(
    p_ggv: np.ndarray,
    ax_max_machines: np.ndarray,
    v_max: float,
    radii: np.ndarray,
    el_lengths: np.ndarray,
    v_start: float,
    drag_coeff: float,
    m_veh: float,
    op_mode: str,
    mu: np.ndarray = None,
    v_end: float = None,
    dyn_model_exp: float = 1.0,
) -> np.ndarray:
    # ------------------------------------------------------------------------------------------------------------------
    # FORWARD BACKWARD SOLVER ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # handle mu
    if mu is None:
        mu = np.ones(radii.size)
        mu_mean = 1.0
    else:
        mu_mean = np.mean(mu)

    # run through all the points and check for possible lateral acceleration
    if op_mode == "ggv":
        # in ggv mode all ggvs are equal -> we can use the first one
        ay_max_global = mu_mean * np.amin(
            p_ggv[0, :, 2]
        )  # get first lateral acceleration estimate
        vx_profile = np.sqrt(
            ay_max_global * radii
        )  # get first velocity profile estimate

        ay_max_curr = mu * np.interp(vx_profile, p_ggv[0, :, 0], p_ggv[0, :, 2])
        vx_profile = np.sqrt(np.multiply(ay_max_curr, radii))

    else:
        # in loc_gg mode all ggvs consist of a single line due to the missing velocity dependency, mu is None in this
        # case
        vx_profile = np.sqrt(
            p_ggv[:, 0, 2] * radii
        )  # get first velocity profile estimate

    # cut vx_profile to car's top speed
    vx_profile[vx_profile > v_max] = v_max

    # consider v_start
    if vx_profile[0] > v_start:
        vx_profile[0] = v_start

    # calculate acceleration profile
    vx_profile = __solver_fb_acc_profile(
        p_ggv=p_ggv,
        ax_max_machines=ax_max_machines,
        v_max=v_max,
        radii=radii,
        el_lengths=el_lengths,
        mu=mu,
        vx_profile=vx_profile,
        backwards=False,
        dyn_model_exp=dyn_model_exp,
        drag_coeff=drag_coeff,
        m_veh=m_veh,
    )

    # consider v_end
    if v_end is not None and vx_profile[-1] > v_end:
        vx_profile[-1] = v_end

    # calculate deceleration profile
    vx_profile = __solver_fb_acc_profile(
        p_ggv=p_ggv,
        ax_max_machines=ax_max_machines,
        v_max=v_max,
        radii=radii,
        el_lengths=el_lengths,
        mu=mu,
        vx_profile=vx_profile,
        backwards=True,
        dyn_model_exp=dyn_model_exp,
        drag_coeff=drag_coeff,
        m_veh=m_veh,
    )

    return vx_profile


def __solver_fb_closed(
    p_ggv: np.ndarray,
    ax_max_machines: np.ndarray,
    v_max: float,
    radii: np.ndarray,
    el_lengths: np.ndarray,
    drag_coeff: float,
    m_veh: float,
    op_mode: str,
    mu: np.ndarray = None,
    dyn_model_exp: float = 1.0,
) -> np.ndarray:
    # ------------------------------------------------------------------------------------------------------------------
    # FORWARD BACKWARD SOLVER ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    no_points = radii.size

    # handle mu
    if mu is None:
        mu = np.ones(no_points)
        mu_mean = 1.0
    else:
        mu_mean = np.mean(mu)

    # run through all the points and check for possible lateral acceleration
    if op_mode == "ggv":
        # in ggv mode all ggvs are equal -> we can use the first one
        ay_max_global = mu_mean * np.amin(
            p_ggv[0, :, 2]
        )  # get first lateral acceleration estimate
        vx_profile = np.sqrt(
            ay_max_global * radii
        )  # get first velocity estimate (radii must be positive!)

        # iterate until the initial velocity profile converges (break after max. 100 iterations)
        converged = False

        for _i in range(100):
            vx_profile_prev_iteration = vx_profile

            ay_max_curr = mu * np.interp(vx_profile, p_ggv[0, :, 0], p_ggv[0, :, 2])
            vx_profile = np.sqrt(np.multiply(ay_max_curr, radii))

            # break the loop if the maximum change of the velocity profile was below 0.5%
            if np.max(np.abs(vx_profile / vx_profile_prev_iteration - 1.0)) < 0.005:
                converged = True
                break

        if not converged:
            print(
                "The initial vx profile did not converge after 100 iterations, please check radii and ggv!"
            )

    else:
        # in loc_gg mode all ggvs consist of a single line due to the missing velocity dependency, mu is None in this
        # case
        vx_profile = np.sqrt(
            p_ggv[:, 0, 2] * radii
        )  # get first velocity estimate (radii must be positive!)

    # cut vx_profile to car's top speed
    vx_profile[vx_profile > v_max] = v_max

    """We need to calculate the speed profile for two laps to get the correct starting and ending velocity."""

    # double arrays
    vx_profile_double = np.concatenate((vx_profile, vx_profile), axis=0)
    radii_double = np.concatenate((radii, radii), axis=0)
    el_lengths_double = np.concatenate((el_lengths, el_lengths), axis=0)
    mu_double = np.concatenate((mu, mu), axis=0)
    p_ggv_double = np.concatenate((p_ggv, p_ggv), axis=0)

    # calculate acceleration profile
    vx_profile_double = __solver_fb_acc_profile(
        p_ggv=p_ggv_double,
        ax_max_machines=ax_max_machines,
        v_max=v_max,
        radii=radii_double,
        el_lengths=el_lengths_double,
        mu=mu_double,
        vx_profile=vx_profile_double,
        backwards=False,
        dyn_model_exp=dyn_model_exp,
        drag_coeff=drag_coeff,
        m_veh=m_veh,
    )

    # use second lap of acceleration profile
    vx_profile_double = np.concatenate(
        (vx_profile_double[no_points:], vx_profile_double[no_points:]), axis=0
    )

    # calculate deceleration profile
    vx_profile_double = __solver_fb_acc_profile(
        p_ggv=p_ggv_double,
        ax_max_machines=ax_max_machines,
        v_max=v_max,
        radii=radii_double,
        el_lengths=el_lengths_double,
        mu=mu_double,
        vx_profile=vx_profile_double,
        backwards=True,
        dyn_model_exp=dyn_model_exp,
        drag_coeff=drag_coeff,
        m_veh=m_veh,
    )

    # use second lap of deceleration profile
    vx_profile = vx_profile_double[no_points:]

    return vx_profile


def __solver_fb_acc_profile(
    p_ggv: np.ndarray,
    ax_max_machines: np.ndarray,
    v_max: float,
    radii: np.ndarray,
    el_lengths: np.ndarray,
    mu: np.ndarray,
    vx_profile: np.ndarray,
    drag_coeff: float,
    m_veh: float,
    dyn_model_exp: float = 1.0,
    backwards: bool = False,
) -> np.ndarray:
    # ------------------------------------------------------------------------------------------------------------------
    # PREPARATIONS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    no_points = vx_profile.size

    # check for reversed direction
    if backwards:
        radii_mod = np.flipud(radii)
        el_lengths_mod = np.flipud(el_lengths)
        mu_mod = np.flipud(mu)
        vx_profile = np.flipud(vx_profile)
        mode = "decel_backw"
    else:
        radii_mod = radii
        el_lengths_mod = el_lengths
        mu_mod = mu
        mode = "accel_forw"

    # ------------------------------------------------------------------------------------------------------------------
    # SEARCH START POINTS FOR ACCELERATION PHASES ----------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    vx_diffs = np.diff(vx_profile)
    acc_inds = np.where(vx_diffs > 0.0)[
        0
    ]  # indices of points with positive acceleration
    if acc_inds.size != 0:
        # check index diffs -> we only need the first point of every acceleration phase
        acc_inds_diffs = np.diff(acc_inds)
        acc_inds_diffs = np.insert(
            acc_inds_diffs, 0, 2
        )  # first point is always a starting point
        acc_inds_rel = acc_inds[
            acc_inds_diffs > 1
        ]  # starting point indices for acceleration phases
    else:
        acc_inds_rel = []  # if vmax is low and can be driven all the time

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE VELOCITY PROFILE ---------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # cast np.array as a list
    acc_inds_rel = list(acc_inds_rel)

    # while we have indices remaining in the list
    while acc_inds_rel:
        # set index to first list element
        i = acc_inds_rel.pop(0)

        # start from current index and run until either the end of the lap or a termination criterion are reached
        while i < no_points - 1:
            ax_possible_cur = calc_ax_poss(
                vx_start=vx_profile[i],
                radius=radii_mod[i],
                ggv=p_ggv[i],
                ax_max_machines=ax_max_machines,
                mu=mu_mod[i],
                mode=mode,
                dyn_model_exp=dyn_model_exp,
                drag_coeff=drag_coeff,
                m_veh=m_veh,
            )

            vx_possible_next = math.sqrt(
                math.pow(vx_profile[i], 2) + 2 * ax_possible_cur * el_lengths_mod[i]
            )

            if backwards:
                """
                We have to loop the calculation if we are in the backwards iteration (currently just once). This is
                because we calculate the possible ax at a point i which does not necessarily fit for point i + 1
                (which is i - 1 in the real direction). At point i + 1 (or i - 1 in real direction) we have a different
                start velocity (vx_possible_next), radius and mu value while the absolute value of ax remains the same
                in both directions.
                """

                # looping just once at the moment
                for _j in range(1):
                    ax_possible_next = calc_ax_poss(
                        vx_start=vx_possible_next,
                        radius=radii_mod[i + 1],
                        ggv=p_ggv[i + 1],
                        ax_max_machines=ax_max_machines,
                        mu=mu_mod[i + 1],
                        mode=mode,
                        dyn_model_exp=dyn_model_exp,
                        drag_coeff=drag_coeff,
                        m_veh=m_veh,
                    )

                    vx_tmp = math.sqrt(
                        math.pow(vx_profile[i], 2)
                        + 2 * ax_possible_next * el_lengths_mod[i]
                    )

                    if vx_tmp < vx_possible_next:
                        vx_possible_next = vx_tmp
                    else:
                        break

            # save possible next velocity if it is smaller than the current value
            if vx_possible_next < vx_profile[i + 1]:
                vx_profile[i + 1] = vx_possible_next

            i += 1

            # break current acceleration phase if next speed would be higher than the maximum vehicle velocity or if we
            # are at the next acceleration phase start index
            if vx_possible_next > v_max or (acc_inds_rel and i >= acc_inds_rel[0]):
                break

    # ------------------------------------------------------------------------------------------------------------------
    # POSTPROCESSING ---------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # flip output vel_profile if necessary
    if backwards:
        vx_profile = np.flipud(vx_profile)

    return vx_profile


def calc_ax_poss(
    vx_start: float,
    radius: float,
    ggv: np.ndarray,
    mu: float,
    dyn_model_exp: float,
    drag_coeff: float,
    m_veh: float,
    ax_max_machines: np.ndarray = None,
    mode: str = "accel_forw",
) -> float:
    """
    This function returns the possible longitudinal acceleration in the current step/point.

    .. inputs::
    :param vx_start:        [m/s] velocity at current point
    :type vx_start:         float
    :param radius:          [m] radius on which the car is currently driving
    :type radius:           float
    :param ggv:             ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2.
    :type ggv:              np.ndarray
    :param mu:              [-] current friction value
    :type mu:               float
    :param dyn_model_exp:   [-] exponent used in the vehicle dynamics model (usual range [1.0,2.0]).
    :type dyn_model_exp:    float
    :param drag_coeff:      [m2*kg/m3] drag coefficient incl. all constants: drag_coeff = 0.5 * c_w * A_front * rho_air
    :type drag_coeff:       float
    :param m_veh:           [kg] vehicle mass
    :type m_veh:            float
    :param ax_max_machines: longitudinal acceleration limits by the electrical motors: [vx, ax_max_machines]. Velocity
                            in m/s, accelerations in m/s2. They should be handed in without considering drag resistance.
                            Can be set None if using one of the decel modes.
    :type ax_max_machines:  np.ndarray
    :param mode:            [-] operation mode, can be 'accel_forw', 'decel_forw', 'decel_backw'
                            -> determines if machine limitations are considered and if ax should be considered negative
                            or positive during deceleration (for possible backwards iteration)
    :type mode:             str

    .. outputs::
    :return ax_final:       [m/s2] final acceleration from current point to next one
    :rtype ax_final:        float
    """

    # ------------------------------------------------------------------------------------------------------------------
    # PREPARATIONS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # check inputs
    if mode not in ["accel_forw", "decel_forw", "decel_backw"]:
        raise RuntimeError("Unknown operation mode for calc_ax_poss!")

    if mode == "accel_forw" and ax_max_machines is None:
        raise RuntimeError(
            "ax_max_machines is required if operation mode is accel_forw!"
        )

    if ggv.ndim != 2 or ggv.shape[1] != 3:
        raise RuntimeError(
            "ggv must have two dimensions and three columns [vx, ax_max, ay_max]!"
        )

    # ------------------------------------------------------------------------------------------------------------------
    # CONSIDER TIRE POTENTIAL ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate possible and used accelerations (considering tires)
    ax_max_tires = mu * np.interp(vx_start, ggv[:, 0], ggv[:, 1])
    ay_max_tires = mu * np.interp(vx_start, ggv[:, 0], ggv[:, 2])
    ay_used = math.pow(vx_start, 2) / radius

    # during forward acceleration and backward deceleration ax_max_tires must be considered positive, during forward
    # deceleration it must be considered negative
    if mode in ["accel_forw", "decel_backw"] and ax_max_tires < 0.0:
        print(
            "WARNING: Inverting sign of ax_max_tires because it should be positive but was negative!"
        )
        ax_max_tires *= -1.0
    elif mode == "decel_forw" and ax_max_tires > 0.0:
        print(
            "WARNING: Inverting sign of ax_max_tires because it should be negative but was positve!"
        )
        ax_max_tires *= -1.0

    radicand = 1.0 - math.pow(ay_used / ay_max_tires, dyn_model_exp)

    if radicand > 0.0:
        ax_avail_tires = ax_max_tires * math.pow(radicand, 1.0 / dyn_model_exp)
    else:
        ax_avail_tires = 0.0

    # ------------------------------------------------------------------------------------------------------------------
    # CONSIDER MACHINE LIMITATIONS -------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # consider limitations imposed by electrical machines during forward acceleration
    if mode == "accel_forw":
        # interpolate machine acceleration to be able to consider varying gear ratios, efficiencies etc.
        ax_max_machines_tmp = np.interp(
            vx_start, ax_max_machines[:, 0], ax_max_machines[:, 1]
        )
        ax_avail_vehicle = min(ax_avail_tires, ax_max_machines_tmp)
    else:
        ax_avail_vehicle = ax_avail_tires

    # ------------------------------------------------------------------------------------------------------------------
    # CONSIDER DRAG ----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate equivalent longitudinal acceleration of drag force at the current speed
    ax_drag = -math.pow(vx_start, 2) * drag_coeff / m_veh

    # drag reduces the possible acceleration in the forward case and increases it in the backward case
    if mode in ["accel_forw", "decel_forw"]:
        ax_final = ax_avail_vehicle + ax_drag
        # attention: this value will now be negative in forward direction if tire is entirely used for cornering
    else:
        ax_final = ax_avail_vehicle - ax_drag

    return ax_final


def calc_ax_profile(
    vx_profile: np.ndarray, el_lengths: np.ndarray, eq_length_output: bool = False
) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    The function calculates the acceleration profile for a given velocity profile.

    .. inputs::
    :param vx_profile:          array containing the velocity profile used as a basis for the acceleration calculations.
    :type vx_profile:           np.ndarray
    :param el_lengths:          array containing the element lengths between every point of the velocity profile.
    :type el_lengths:           np.ndarray
    :param eq_length_output:    assumes zero acceleration for the last point of the acceleration profile and therefore
                                returns ax_profile with equal length to vx_profile.
    :type eq_length_output:     bool

    .. outputs::
    :return ax_profile:         acceleration profile calculated for the inserted vx_profile.
    :rtype ax_profile:          np.ndarray

    .. notes::
    case eq_length_output is True:
    len(vx_profile) = len(el_lengths) + 1 = len(ax_profile)

    case eq_length_output is False:
    len(vx_profile) = len(el_lengths) + 1 = len(ax_profile) + 1
    """

    # check inputs
    if vx_profile.size != el_lengths.size + 1:
        raise RuntimeError(
            "Array size of vx_profile should be 1 element bigger than el_lengths!"
        )

    # calculate longitudinal acceleration profile array numerically: (v_end^2 - v_beg^2) / 2*s
    if eq_length_output:
        ax_profile = np.zeros(vx_profile.size)
        ax_profile[:-1] = (
            np.power(vx_profile[1:], 2) - np.power(vx_profile[:-1], 2)
        ) / (2 * el_lengths)
    else:
        ax_profile = (np.power(vx_profile[1:], 2) - np.power(vx_profile[:-1], 2)) / (
            2 * el_lengths
        )

    return ax_profile


def calc_t_profile(
    vx_profile: np.ndarray,
    el_lengths: np.ndarray,
    t_start: float = 0.0,
    ax_profile: np.ndarray = None,
) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Calculate a temporal duration profile for a given trajectory.

    .. inputs::
    :param vx_profile:  array containing the velocity profile.
    :type vx_profile:   np.ndarray
    :param el_lengths:  array containing the element lengths between every point of the velocity profile.
    :type el_lengths:   np.ndarray
    :param t_start:     start time in seconds added to first array element.
    :type t_start:      float
    :param ax_profile:  acceleration profile fitting to the velocity profile.
    :type ax_profile:   np.ndarray

    .. outputs::
    :return t_profile:  time profile for the given velocity profile.
    :rtype t_profile:   np.ndarray

    .. notes::
    len(el_lengths) + 1 = len(t_profile)

    len(vx_profile) and len(ax_profile) must be >= len(el_lengths) as the temporal duration from one point to the next
    is only calculated based on the previous point.
    """

    # check inputs
    if vx_profile.size < el_lengths.size:
        raise RuntimeError(
            "vx_profile and el_lenghts must have at least the same length!"
        )

    if ax_profile is not None and ax_profile.size < el_lengths.size:
        raise RuntimeError(
            "ax_profile and el_lenghts must have at least the same length!"
        )

    # calculate acceleration profile if required
    if ax_profile is None:
        ax_profile = calc_ax_profile(
            vx_profile=vx_profile, el_lengths=el_lengths, eq_length_output=False
        )

    # calculate temporal duration of every step between two points
    no_points = el_lengths.size
    t_steps = np.zeros(no_points)

    for i in range(no_points):
        if not math.isclose(ax_profile[i], 0.0):
            t_steps[i] = (
                -vx_profile[i]
                + math.sqrt(
                    (math.pow(vx_profile[i], 2) + 2 * ax_profile[i] * el_lengths[i])
                )
            ) / ax_profile[i]

        else:  # ax == 0.0
            t_steps[i] = el_lengths[i] / vx_profile[i]

    # calculate temporal duration profile out of steps
    t_profile = np.insert(np.cumsum(t_steps), 0, 0.0) + t_start

    return t_profile


def result_plots(
    plot_opts: dict,
    width_veh_opt: float,
    width_veh_real: float,
    refline: np.ndarray,
    bound1_imp: np.ndarray,
    bound2_imp: np.ndarray,
    bound1_interp: np.ndarray,
    bound2_interp: np.ndarray,
    trajectory: np.ndarray,
    cones_left: np.ndarray,
    cones_right: np.ndarray,
    bound_left: np.ndarray,
    bound_right: np.ndarray,
) -> None:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    This function plots several figures containing relevant trajectory information after trajectory optimization.

    Inputs:
    plot_opts:      dict containing the information which figures to plot
    width_veh_opt:  vehicle width used during optimization in m
    width_veh_real: real vehicle width in m
    refline:        contains the reference line coordinates [x_m, y_m]
    bound1_imp:     first track boundary (as imported) (mostly right) [x_m, y_m]
    bound2_imp:     second track boundary (as imported) (mostly left) [x_m, y_m]
    bound1_interp:  first track boundary (interpolated) (mostly right) [x_m, y_m]
    bound2_interp:  second track boundary (interpolated) (mostly left) [x_m, y_m]
    trajectory:     trajectory data [s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2]
    cones_left:     left cone coordinates [x_m, y_m]
    cones_right:    right cone coordinates [x_m, y_m]
    """

    if plot_opts["raceline"]:
        # calculate vehicle boundary points (including safety margin in vehicle width)
        normvec_normalized_opt = calc_normal_vectors(trajectory[:, 3])

        veh_bound1_virt = (
            trajectory[:, 1:3] + normvec_normalized_opt * width_veh_opt / 2
        )
        veh_bound2_virt = (
            trajectory[:, 1:3] - normvec_normalized_opt * width_veh_opt / 2
        )

        veh_bound1_real = (
            trajectory[:, 1:3] + normvec_normalized_opt * width_veh_real / 2
        )
        veh_bound2_real = (
            trajectory[:, 1:3] - normvec_normalized_opt * width_veh_real / 2
        )

        point1_arrow = refline[0]
        point2_arrow = refline[2]
        vec_arrow = point2_arrow - point1_arrow

        refline_plot = np.vstack((refline, refline[0]))

        # plot track including optimized path
        plt.figure("Minimum curvature racing line")
        plt.title("Minimum curvature racing line")
        plt.grid()

        mincurv_path_1 = plt.plot(
            refline_plot[:, 0],
            refline_plot[:, 1],
            color=normalize_color((0, 0, 0)),
            linestyle="dashed",
            linewidth=0.7,
            marker="o",
            markersize=2,
            markerfacecolor=normalize_color((0, 0, 0)),
        )
        mincurv_path_2 = plt.plot(
            veh_bound1_virt[:, 0],
            veh_bound1_virt[:, 1],
            color=normalize_color((255, 0, 255)),
            linewidth=0.5,
        )
        plt.plot(
            veh_bound2_virt[:, 0],
            veh_bound2_virt[:, 1],
            color=normalize_color((255, 0, 255)),
            linewidth=0.5,
        )
        mincurv_path_4 = plt.plot(
            veh_bound1_real[:, 0],
            veh_bound1_real[:, 1],
            color=normalize_color((0, 255, 0)),
            linewidth=0.5,
        )
        plt.plot(
            veh_bound2_real[:, 0],
            veh_bound2_real[:, 1],
            color=normalize_color((0, 255, 0)),
            linewidth=0.5,
        )
        mincurv_path_6 = plt.plot(
            trajectory[:, 1],
            trajectory[:, 2],
            color=normalize_color((255, 0, 0)),
            linewidth=0.7,
        )
        mincurv_path_7 = plt.scatter(
            cones_left[:, 0],
            cones_left[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        plt.scatter(
            cones_right[:, 0],
            cones_right[:, 1],
            color=normalize_color((255, 0, 0)),
            marker="o",
            s=25,
        )
        mincurv_path_9 = plt.plot(
            bound_left[:, 0],
            bound_left[:, 1],
            color=normalize_color((0, 0, 255)),
            linewidth=1.0,
        )
        mincurv_path_10 = plt.plot(
            bound_right[:, 0],
            bound_right[:, 1],
            color=normalize_color((230, 245, 0)),
            linewidth=1.0,
        )
        mincurv_path_11 = plt.arrow(
            point1_arrow[0],
            point1_arrow[1],
            vec_arrow[0],
            vec_arrow[1],
            width=0.5,
            head_width=1.0,
            head_length=1.0,
            fc="g",
            ec="g",
        )

        plt.legend(
            [
                mincurv_path_1[0],
                mincurv_path_2[0],
                mincurv_path_4[0],
                mincurv_path_6[0],
                mincurv_path_7,
                mincurv_path_9[0],
                mincurv_path_10[0],
                mincurv_path_11,
            ],
            [
                "Reference line",
                "Vehicle boundary (virtual)",
                "Vehicle boundary (real)",
                "Minimum curvature path",
                "Cones",
                "Left boundary",
                "Right boundary",
                "Driving direction",
            ],
            handler_map={
                mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)
            },
        )

        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.xlabel("x-distance from original car position in m")
        plt.ylabel("y-distance from original car position in m")
        plt.show()

    if plot_opts["raceline_curv"]:
        # plot curvature profile
        plt.figure("Curvature profile")
        plt.title("Curvature profile")
        plt.plot(trajectory[:-1, 0], trajectory[:-1, 4])
        plt.grid()
        plt.xlabel("distance in m")
        plt.ylabel("curvature in rad/m")
        plt.show()

    if plot_opts["racetraj_vel_3d"]:
        scale_x = 1.0
        scale_y = 1.0
        scale_z = 0.3  # scale z axis such that it does not appear stretched

        # create 3d plot
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")

        # recast get_proj function to use scaling factors for the axes
        ax.get_proj = lambda: np.dot(
            Axes3D.get_proj(ax), np.diag([scale_x, scale_y, scale_z, 1.0])
        )

        # plot raceline and boundaries
        ax.plot(refline[:, 0], refline[:, 1], "k--", linewidth=0.7)
        ax.plot(bound1_interp[:, 0], bound1_interp[:, 1], 0.0, "k-", linewidth=0.7)
        ax.plot(bound2_interp[:, 0], bound2_interp[:, 1], 0.0, "k-", linewidth=0.7)
        ax.plot(trajectory[:, 1], trajectory[:, 2], "r-", linewidth=0.7)

        ax.grid()
        ax.set_aspect("equalxy")
        ax.set_xlabel("east in m")
        ax.set_ylabel("north in m")

        # plot velocity profile in 3D
        ax.plot(trajectory[:, 1], trajectory[:, 2], trajectory[:, 5], color="k")
        ax.set_zlabel("velocity in m/s")

        # plot vertical lines visualizing acceleration and deceleration zones
        ind_stepsize = int(
            np.round(
                plot_opts["racetraj_vel_3d_stepsize"] / trajectory[1, 0]
                - trajectory[0, 0]
            )
        )
        if ind_stepsize < 1:
            ind_stepsize = 1

        cur_ind = 0
        no_points_traj_vdc = np.shape(trajectory)[0]

        while cur_ind < no_points_traj_vdc - 1:
            x_tmp = [trajectory[cur_ind, 1], trajectory[cur_ind, 1]]
            y_tmp = [trajectory[cur_ind, 2], trajectory[cur_ind, 2]]
            z_tmp = [
                0.0,
                trajectory[cur_ind, 5],
            ]  # plot line with height depending on velocity

            # get proper color for line depending on acceleration
            if trajectory[cur_ind, 6] > 0.0:
                col = "g"
            elif trajectory[cur_ind, 6] < 0.0:
                col = "r"
            else:
                col = "gray"

            # plot line
            ax.plot(x_tmp, y_tmp, z_tmp, color=col)

            # increment index
            cur_ind += ind_stepsize

        plt.show()

    if plot_opts["spline_normals"]:
        plt.figure()

        plt.plot(refline[:, 0], refline[:, 1], "k-")
        for i in range(bound1_interp.shape[0]):
            temp = np.vstack((bound1_interp[i], bound2_interp[i]))
            plt.plot(temp[:, 0], temp[:, 1], "r-", linewidth=0.7)

        plt.grid()
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.xlabel("east in m")
        plt.ylabel("north in m")

        plt.show()


def check_traj(
    reftrack: np.ndarray,
    reftrack_normvec_normalized: np.ndarray,
    trajectory: np.ndarray,
    ggv: np.ndarray,
    ax_max_machines: np.ndarray,
    v_max: float,
    length_veh: float,
    width_veh: float,
    debug: bool,
    dragcoeff: float,
    mass_veh: float,
    curvlim: float,
) -> tuple:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    This function checks the generated trajectory in regards of minimum distance to the boundaries and maximum
    curvature and accelerations.

    Inputs:
    reftrack:           track [x_m, y_m, w_tr_right_m, w_tr_left_m]
    reftrack_normvec_normalized: normalized normal vectors on the reference line [x_m, y_m]
    trajectory:         trajectory to be checked [s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2]
    ggv:                ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2.
    ax_max_machines:    longitudinal acceleration limits by the electrical motors: [vx, ax_max_machines]. Velocity
                        in m/s, accelerations in m/s2. They should be handed in without considering drag resistance.
    v_max:              Maximum longitudinal speed in m/s.
    length_veh:         vehicle length in m
    width_veh:          vehicle width in m
    debug:              boolean showing if debug messages should be printed
    dragcoeff:          [m2*kg/m3] drag coefficient containing c_w_A * rho_air * 0.5
    mass_veh:           [kg] mass
    curvlim:            [rad/m] maximum drivable curvature

    Outputs:
    bound_r:            right track boundary [x_m, y_m]
    bound_l:            left track boundary [x_m, y_m]
    """

    # ------------------------------------------------------------------------------------------------------------------
    # CHECK VEHICLE EDGES FOR MINIMUM DISTANCE TO TRACK BOUNDARIES -----------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate boundaries and interpolate them to small stepsizes (currently linear interpolation)
    bound_r = reftrack[:, :2] + reftrack_normvec_normalized * np.expand_dims(
        reftrack[:, 2], 1
    )
    bound_l = reftrack[:, :2] - reftrack_normvec_normalized * np.expand_dims(
        reftrack[:, 3], 1
    )

    # check boundaries for vehicle edges
    bound_r_tmp = np.column_stack((bound_r, np.zeros((bound_r.shape[0], 2))))
    bound_l_tmp = np.column_stack((bound_l, np.zeros((bound_l.shape[0], 2))))

    bound_r_interp = interp_track_2(reftrack=bound_r_tmp, stepsize_approx=1.0)[0]
    bound_l_interp = interp_track_2(reftrack=bound_l_tmp, stepsize_approx=1.0)[0]

    # calculate minimum distances of every trajectory point to the boundaries
    min_dists = calc_min_bound_dists(
        trajectory=trajectory,
        bound1=bound_r_interp,
        bound2=bound_l_interp,
        length_veh=length_veh,
        width_veh=width_veh,
    )

    # calculate overall minimum distance
    min_dist = np.amin(min_dists)

    # warn if distance falls below a safety margin of 1.0 m
    if min_dist < 1.0:
        print(
            "WARNING: Minimum distance to boundaries is estimated to %.2fm. Keep in mind that the distance can also"
            " lie on the outside of the track!" % min_dist
        )
    elif debug:
        print(
            "INFO: Minimum distance to boundaries is estimated to %.2fm. Keep in mind that the distance can also lie"
            " on the outside of the track!" % min_dist
        )

    # ------------------------------------------------------------------------------------------------------------------
    # CHECK FINAL TRAJECTORY FOR MAXIMUM CURVATURE ---------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # check maximum (absolute) curvature
    if np.amax(np.abs(trajectory[:, 4])) > curvlim:
        print(
            "WARNING: Curvature limit is exceeded: %.3frad/m"
            % np.amax(np.abs(trajectory[:, 4]))
        )

    # ------------------------------------------------------------------------------------------------------------------
    # CHECK FINAL TRAJECTORY FOR MAXIMUM ACCELERATIONS -----------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if ggv is not None:
        # transform curvature kappa into corresponding radii (abs because curvature has a sign in our convention)
        radii = np.abs(
            np.divide(
                1.0,
                trajectory[:, 4],
                out=np.full(trajectory.shape[0], np.inf),
                where=trajectory[:, 4] != 0,
            )
        )

        # check max. lateral accelerations
        ay_profile = np.divide(np.power(trajectory[:, 5], 2), radii)

        if np.any(ay_profile > np.amax(ggv[:, 2]) + 0.1):
            print(
                "WARNING: Lateral ggv acceleration limit is exceeded: %.2fm/s2"
                % np.amax(ay_profile)
            )

        # check max. longitudinal accelerations (consider that drag is included in the velocity profile!)
        ax_drag = -np.power(trajectory[:, 5], 2) * dragcoeff / mass_veh
        ax_wo_drag = trajectory[:, 6] - ax_drag

        if np.any(ax_wo_drag > np.amax(ggv[:, 1]) + 0.1):
            print(
                "WARNING: Longitudinal ggv acceleration limit (positive) is exceeded: %.2fm/s2"
                % np.amax(ax_wo_drag)
            )

        if np.any(ax_wo_drag < np.amin(-ggv[:, 1]) - 0.1):
            print(
                "WARNING: Longitudinal ggv acceleration limit (negative) is exceeded: %.2fm/s2"
                % np.amin(ax_wo_drag)
            )

        # check total acceleration
        a_tot = np.sqrt(np.power(ax_wo_drag, 2) + np.power(ay_profile, 2))

        if np.any(a_tot > np.amax(ggv[:, 1:]) + 0.1):
            print(
                "WARNING: Total ggv acceleration limit is exceeded: %.2fm/s2"
                % np.amax(a_tot)
            )

    else:
        print(
            "WARNING: Since ggv-diagram was not given the according checks cannot be performed!"
        )

    if ax_max_machines is not None:
        # check max. longitudinal accelerations (consider that drag is included in the velocity profile!)
        ax_drag = -np.power(trajectory[:, 5], 2) * dragcoeff / mass_veh
        ax_wo_drag = trajectory[:, 6] - ax_drag

        if np.any(ax_wo_drag > np.amax(ax_max_machines[:, 1]) + 0.1):
            print(
                "WARNING: Longitudinal acceleration machine limits are exceeded: %.2fm/s2"
                % np.amax(ax_wo_drag)
            )

    # ------------------------------------------------------------------------------------------------------------------
    # CHECK FINAL TRAJECTORY FOR MAXIMUM VELOCITY ----------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if np.any(trajectory[:, 5] > v_max + 0.1):
        print(
            "WARNING: Maximum velocity of final trajectory exceeds the maximal velocity of the vehicle: %.2fm/s!"
            % np.amax(trajectory[:, 5])
        )

    return bound_r, bound_l


def calc_min_bound_dists(
    trajectory: np.ndarray,
    bound1: np.ndarray,
    bound2: np.ndarray,
    length_veh: float,
    width_veh: float,
) -> np.ndarray:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    Calculate minimum distance between vehicle and track boundaries for every trajectory point. Vehicle dimensions are
    taken into account for this calculation. Vehicle orientation is assumed to be the same as the heading of the
    trajectory.

    Inputs:
    trajectory:     array containing the trajectory information. Required are x, y, psi for every point
    bound1/2:       arrays containing the track boundaries [x, y]
    length_veh:     real vehicle length in m
    width_veh:      real vehicle width in m

    Outputs:
    min_dists:      minimum distance to boundaries for every trajectory point
    """

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE MINIMUM DISTANCES --------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    bounds = np.vstack((bound1, bound2))

    # calculate static vehicle edge positions [x, y] for psi = 0
    fl = np.array([-width_veh / 2, length_veh / 2])
    fr = np.array([width_veh / 2, length_veh / 2])
    rl = np.array([-width_veh / 2, -length_veh / 2])
    rr = np.array([width_veh / 2, -length_veh / 2])

    # loop through all the raceline points
    min_dists = np.zeros(trajectory.shape[0])
    mat_rot = np.zeros((2, 2))

    for i in range(trajectory.shape[0]):
        mat_rot[0, 0] = math.cos(trajectory[i, 3])
        mat_rot[0, 1] = -math.sin(trajectory[i, 3])
        mat_rot[1, 0] = math.sin(trajectory[i, 3])
        mat_rot[1, 1] = math.cos(trajectory[i, 3])

        # calculate positions of vehicle edges
        fl_ = trajectory[i, 1:3] + np.matmul(mat_rot, fl)
        fr_ = trajectory[i, 1:3] + np.matmul(mat_rot, fr)
        rl_ = trajectory[i, 1:3] + np.matmul(mat_rot, rl)
        rr_ = trajectory[i, 1:3] + np.matmul(mat_rot, rr)

        # get minimum distances of vehicle edges to any boundary point
        fl__mindist = np.sqrt(
            np.power(bounds[:, 0] - fl_[0], 2) + np.power(bounds[:, 1] - fl_[1], 2)
        )
        fr__mindist = np.sqrt(
            np.power(bounds[:, 0] - fr_[0], 2) + np.power(bounds[:, 1] - fr_[1], 2)
        )
        rl__mindist = np.sqrt(
            np.power(bounds[:, 0] - rl_[0], 2) + np.power(bounds[:, 1] - rl_[1], 2)
        )
        rr__mindist = np.sqrt(
            np.power(bounds[:, 0] - rr_[0], 2) + np.power(bounds[:, 1] - rr_[1], 2)
        )

        # save overall minimum distance of current vehicle position
        min_dists[i] = np.amin((fl__mindist, fr__mindist, rl__mindist, rr__mindist))

    return min_dists


def calc_normal_vectors(psi: np.ndarray) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Use heading to calculate normalized (i.e. unit length) normal vectors. Normal vectors point in direction psi - pi/2.

    .. inputs::
    :param psi:                     array containing the heading of every point (north up, range [-pi,pi[).
    :type psi:                      np.ndarray

    .. outputs::
    :return normvec_normalized:     unit length normal vectors for every point [x, y].
    :rtype normvec_normalized:      np.ndarray

    .. notes::
    len(psi) = len(normvec_normalized)
    """

    # calculate normal vectors
    normvec_normalized = -calc_normal_vectors_ahead(psi=psi)

    return normvec_normalized


def calc_normal_vectors_ahead(psi: np.ndarray) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Use heading to calculate normalized (i.e. unit length) normal vectors. Normal vectors point in direction psi + pi/2.

    .. inputs::
    :param psi:                     array containing the heading of every point (north up, range [-pi,pi[).
    :type psi:                      np.ndarray

    .. outputs::
    :return normvec_normalized:     unit length normal vectors for every point [x, y].
    :rtype normvec_normalized:      np.ndarray

    .. notes::
    len(psi) = len(normvec_normalized)
    """

    # calculate tangent vectors
    tangvec_normalized = calc_tangent_vectors(psi=psi)

    # find normal vectors
    normvec_normalized = np.stack(
        (-tangvec_normalized[:, 1], tangvec_normalized[:, 0]), axis=1
    )

    return normvec_normalized


def calc_tangent_vectors(psi: np.ndarray) -> np.ndarray:
    """
    author:
    Alexander Heilmeier

    .. description::
    Use heading to calculate normalized (i.e. unit length) tangent vectors.

    .. inputs::
    :param psi:                     array containing the heading of every point (north up, range [-pi,pi[).
    :type psi:                      np.ndarray

    .. outputs::
    :return tangvec_normalized:     unit length tangent vectors for every point [x, y].
    :rtype tangvec_normalized:      np.ndarray

    .. notes::
    len(psi) = len(tangvec_normalized)
    """

    psi_ = np.copy(psi)

    # remap psi_vel to x-axis
    psi_ += math.pi / 2
    psi_ = normalize_psi(psi_)

    # get normalized tangent vectors
    tangvec_normalized = np.zeros((psi_.size, 2))
    tangvec_normalized[:, 0] = np.cos(psi_)
    tangvec_normalized[:, 1] = np.sin(psi_)

    return tangvec_normalized


def interp_track_2(reftrack: np.ndarray, stepsize_approx: float = 1.0) -> np.ndarray:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    Use linear interpolation between track points to create new points with equal distances.

    Inputs:
    reftrack:           array containing the track information that shell be interpolated [x, y, w_tr_right, w_tr_left].
    stepsize_approx:    desired stepsize for the interpolation

    Outputs:
    reftrack_interp:    interpolated reference track (unclosed)
    """

    # ------------------------------------------------------------------------------------------------------------------
    # FUNCTION BODY ----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    reftrack_cl = np.vstack((reftrack, reftrack[0]))

    # calculate element lengths (euclidian distance)
    el_lenghts = np.sqrt(
        np.sum(np.power(np.diff(reftrack_cl[:, :2], axis=0), 2), axis=1)
    )

    # sum up total distance (from start) to every element
    dists_cum = np.cumsum(el_lenghts)
    dists_cum = np.insert(dists_cum, 0, 0.0)

    # calculate desired lenghts depending on specified stepsize (+1 because last element is included)
    no_points_interp = math.ceil(dists_cum[-1] / stepsize_approx) + 1
    dists_interp = np.linspace(0.0, dists_cum[-1], no_points_interp)

    # interpolate closed track points
    reftrack_interp_cl = np.zeros((no_points_interp, 4))
    reftrack_interp_cl[:, 0] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 0])
    reftrack_interp_cl[:, 1] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 1])
    reftrack_interp_cl[:, 2] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 2])
    reftrack_interp_cl[:, 3] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 3])

    # remove closed points
    reftrack_interp = reftrack_interp_cl[:-1]

    return reftrack_interp


def normalize_color(color: tuple) -> tuple:
    """
    Created by:
    Kwinten Mortier

    .. description::
    Normalize a tuple representing a RGB color to be used in matplotlib.

    .. inputs::
    :param color:                   tuple containing the RGB values of a color
    :type color:                    tuple

    .. outputs::
    :return color_normalized:       tuple containing the normalized RGB values of a color
    :rtype color_normalized:        tuple
    """

    # normalize the color
    color_normalized = (color[0] / 255, color[1] / 255, color[2] / 255)

    return color_normalized
