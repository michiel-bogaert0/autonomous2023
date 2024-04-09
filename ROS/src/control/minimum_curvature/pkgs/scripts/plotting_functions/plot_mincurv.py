import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.legend_handler import HandlerPatch
from mpl_toolkits.mplot3d import Axes3D
from plotting_functions.plotting_utils import (
    make_legend_arrow,
    normalize_color,
    save_plot_to_folder,
)


def plot_preprocessing(
    cones_left_: np.ndarray,
    cones_right_: np.ndarray,
    initial_poses_: np.ndarray,
    trajectory_interp_cl_: np.ndarray,
    bound_left_interp_cl_: np.ndarray,
    bound_right_interp_cl_: np.ndarray,
    trajectory_smoothed_cl_: np.ndarray,
    bound_left_smoothed_cl_: np.ndarray,
    bound_right_smoothed_cl_: np.ndarray,
    prepped_track_: np.ndarray,
    new_bound_left_: np.ndarray,
    new_bound_right_: np.ndarray,
    kappa_ref_: np.ndarray,
    dkappa_ref_: np.ndarray,
    kappa_prepped_: np.ndarray,
    dkappa_prepped_: np.ndarray,
    s_ref_: np.ndarray,
    s_prepped_: np.ndarray,
    plot_options: dict,
    vx_profile_prepped: np.ndarray = None,
    ax_profile_prepped: np.ndarray = None,
    raceline_prepped: np.ndarray = None,
):
    """
    author:
    Kwinten Mortier

    .. description::
    This function plots the preprocessing results.

    .. inputs::
    :param cones_left:                array containing the left track cones [x, y] (unclosed!).
    :type cones_left:                 np.ndarray
    :param cones_right:               array containing the right track cones [x, y] (unclosed!).
    :type cones_right:                np.ndarray
    :param initial_poses:             array containing the poses of the trajectory from pathplanning [x, y] (unclosed!).
    :type initial_poses:              np.ndarray
    :param trajectory_interp_cl:      array containing the interpolated trajectory [x, y] (closed!).
    :type trajectory_interp_cl:       np.ndarray
    :param bound_left_interp_cl:      array containing the interpolated left boundary [x, y] (closed!).
    :type bound_left_interp_cl:       np.ndarray
    :param bound_right_interp_cl:     array containing the interpolated right boundary [x, y] (closed!).
    :type bound_right_interp_cl:      np.ndarray
    :param trajectory_smoothed_cl:    array containing the smoothed trajectory [x, y] (closed!).
    :type trajectory_smoothed_cl:     np.ndarray
    :param bound_left_smoothed_cl:    array containing the smoothed left boundary [x, y] (closed!).
    :type bound_left_smoothed_cl:     np.ndarray
    :param bound_right_smoothed_cl:   array containing the smoothed right boundary [x, y] (closed!).
    :type bound_right_smoothed_cl:    np.ndarray
    :param prepped_track:             array containing the preprocessed track, i.e. a reference line and the
                                        according track widths to the left and to the right [x, y, w_tr_left, w_tr_right]
                                        (unit is meter, unclosed!)
    :type prepped_track:              np.ndarray
    :param new_bound_left:            array containing the new left boundary [x, y] (unclosed!).
    :type new_bound_left:             np.ndarray
    :param new_bound_right:           array containing the new right boundary [x, y] (unclosed!).
    :type new_bound_right:            np.ndarray
    :param kappa_ref:                 curvature for every point of the reference track [x, y] (unit is 1/m, unclosed!).
    :type kappa_ref:                  np.ndarray
    :param dkappa_ref:                derivative of curvature for every point of the reference track [x, y]
                                        (unit is 1/m^2, unclosed!)
    :type dkappa_ref:                 np.ndarray
    :param kappa_prepped:             curvature for every point of the preprocessed track [x, y] (unit is 1/m, unclosed!).
    :type kappa_prepped:              np.ndarray
    :param dkappa_prepped:            derivative of curvature for every point of the preprocessed track [x, y]
                                        (unit is 1/m^2, unclosed!)
    :type dkappa_prepped:             np.ndarray
    :param s_ref:                     array containing the s-values for the reference track.
    :type s_ref:                      np.ndarray
    :param s_prepped:                 array containing the s-values for the preprocessed track.
    :type s_prepped:                  np.ndarray
    :param plot_options:              dictionary containing the plot options to determine which plots to show.
    :type plot_options:               dict
    :param vx_profile_prepped:        array containing the velocity profile of the preprocessed track.
    :type vx_profile_prepped:         np.ndarray
    :param ax_profile_prepped:        array containing the acceleration profile of the preprocessed track.
    :type ax_profile_prepped:         np.ndarray
    :param raceline_prepped:          array containing the raceline of the preprocessed track.
    :type raceline_prepped:           np.ndarray

    .. notes::
    PREPROCESSING PLOTS:
    - FIG 1: Initial data
    - FIG 2: Quadratic interpolation
    - FIG 3: BSpline smoothing
    - FIG 4: Boundary distance estimation
    - FIG 5: Preprocessed track
    - FIG 6: Curvature comparison reference track and preprocessed track
    - FIG 7: Curvature derivative comparison reference track and preprocessed track

    OPTIONAL PREPROCESSING PLOTS FOR VELOCITY AND ACCELERATION PROFILES (if calc_vel is True):
    - FIG 8: Velocity profile preprocessed track
    - FIG 9: Velocity profile preprocessed track 3D
    - FIG 10: Acceleration profile preprocessed track
    """

    # ------------------------------------------------------------------------------------------------------------------
    # COPY INPUTS ------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    cones_left = np.copy(cones_left_)
    cones_right = np.copy(cones_right_)
    initial_poses = np.copy(initial_poses_)
    trajectory_interp_cl = np.copy(trajectory_interp_cl_)
    bound_left_interp_cl = np.copy(bound_left_interp_cl_)
    bound_right_interp_cl = np.copy(bound_right_interp_cl_)
    trajectory_smoothed_cl = np.copy(trajectory_smoothed_cl_)
    bound_left_smoothed_cl = np.copy(bound_left_smoothed_cl_)
    bound_right_smoothed_cl = np.copy(bound_right_smoothed_cl_)
    prepped_track = np.copy(prepped_track_)
    new_bound_left = np.copy(new_bound_left_)
    new_bound_right = np.copy(new_bound_right_)
    kappa_ref = np.copy(kappa_ref_)
    dkappa_ref = np.copy(dkappa_ref_)
    kappa_prepped = np.copy(kappa_prepped_)
    dkappa_prepped = np.copy(dkappa_prepped_)
    s_ref = np.copy(s_ref_)
    s_prepped = np.copy(s_prepped_)

    # ------------------------------------------------------------------------------------------------------------------
    # CHECKING INPUTS --------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    if (
        (cones_left[0] == cones_left[-1]).all()
        or (cones_right[0] == cones_right[-1]).all()
        or (initial_poses[0] == initial_poses[-1]).all()
        or (prepped_track[0] == prepped_track[-1]).all()
    ):
        raise RuntimeError("Initial data and tracks must be unclosed!")

    if (
        (not np.all(np.isclose(trajectory_interp_cl[0], trajectory_interp_cl[-1])))
        or (not np.all(np.isclose(bound_left_interp_cl[0], bound_left_interp_cl[-1])))
        or (not np.all(np.isclose(bound_right_interp_cl[0], bound_right_interp_cl[-1])))
        or (
            not np.all(
                np.isclose(trajectory_smoothed_cl[0], trajectory_smoothed_cl[-1])
            )
        )
        or (
            not np.all(
                np.isclose(bound_left_smoothed_cl[0], bound_left_smoothed_cl[-1])
            )
        )
        or (
            not np.all(
                np.isclose(bound_right_smoothed_cl[0], bound_right_smoothed_cl[-1])
            )
        )
    ):
        raise RuntimeError("Interpolated and smoothed tracks must be closed!")

    if (
        cones_left.shape[1] != 2
        or cones_right.shape[1] != 2
        or initial_poses.shape[1] != 2
        or bound_left_interp_cl.shape[1] != 2
        or bound_right_interp_cl.shape[1] != 2
        or bound_left_smoothed_cl.shape[1] != 2
        or bound_right_smoothed_cl.shape[1] != 2
        or new_bound_left.shape[1] != 2
        or new_bound_right.shape[1] != 2
    ):
        raise RuntimeError("Initial data and boundaries must have the shape [x, y]!")

    if prepped_track.shape[1] != 4:
        raise RuntimeError("Tracks must have the shape [x, y, w_tr_left, w_tr_right]!")

    if (
        (kappa_ref is not None and kappa_ref[0] == kappa_ref[-1].all())
        or (dkappa_ref is not None and dkappa_ref[0] == dkappa_ref[-1].all())
        or (kappa_prepped is not None and kappa_prepped[0] == kappa_prepped[-1].all())
        or (
            dkappa_prepped is not None and dkappa_prepped[0] == dkappa_prepped[-1].all()
        )
    ):
        raise RuntimeError("Curvature and curvature derivative data must be unclosed!")

    # ------------------------------------------------------------------------------------------------------------------
    # DEFINING VARIABLES -----------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # Closed trajectories and tracks
    initial_poses_cl = np.vstack((initial_poses, initial_poses[0]))
    # prepped_track_cl = np.vstack((prepped_track, prepped_track[0]))

    # Closed boundaries
    new_bound_left_cl = np.vstack((new_bound_left, new_bound_left[0]))
    new_bound_right_cl = np.vstack((new_bound_right, new_bound_right[0]))

    # Arrow variables for the legend (driving direction)
    point1_arrow = initial_poses[0]
    point2_arrow = initial_poses[2]
    vector_arrow = point2_arrow - point1_arrow

    # Determine the plot options
    calc_vel = plot_options["calc_vel"]
    show_plots = plot_options["show_plots"]
    save_plots = plot_options["save_plots"]
    folder_path = plot_options["folder_path"]

    # Closed variables for 3D plot
    if calc_vel:
        vx_profile_prepped_cl = np.hstack((vx_profile_prepped, vx_profile_prepped[0]))
        ax_profile_prepped_cl = np.hstack((ax_profile_prepped, ax_profile_prepped[0]))
        raceline_prepped_cl = np.vstack((raceline_prepped, raceline_prepped[0]))

    ####################################################################################################################
    # PLOTTING FIGURES
    # - FIGURE 1: Initial data
    # - FIGURE 2: Quadratic interpolation
    # - FIGURE 3: BSpline smoothing
    # - FIGURE 4: Boundary distance estimation
    # - FIGURE 5: Preprocessed track
    # - FIGURE 6: Curvature comparison reference track and preprocessed track
    # - FIGURE 7: Curvature derivative comparison reference track and preprocessed track
    #
    # OPTIONAL PLOTTING FIGURES FOR VELOCITY AND ACCELERATION PROFILES (if calc_vel is True)
    # - FIGURE 8: Velocity profile preprocessed track
    # - FIGURE 9: Velocity profile preprocessed track 3D
    # - FIGURE 10: Acceleration profile preprocessed track
    ####################################################################################################################

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 1: Initial data -------------------------------------------------------------------------------------------
    # - Left cones (fig_1_1)
    # - Right cones (fig_1_2)
    # - Initial poses (fig_1_3)
    # - Driving direction arrow (fig_1_4)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure("Initial data")
    plt.grid()
    fig_1_1 = plt.scatter(
        cones_left[:, 0],
        cones_left[:, 1],
        color=normalize_color((0, 0, 255)),
        marker="o",
        s=25,
    )
    fig_1_2 = plt.scatter(
        cones_right[:, 0],
        cones_right[:, 1],
        color=normalize_color((230, 240, 0)),
        marker="o",
        s=25,
    )
    fig_1_3 = plt.plot(
        initial_poses_cl[:, 0],
        initial_poses_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=1,
        linestyle="dashed",
        marker="o",
        markersize=3,
        markerfacecolor=normalize_color((0, 0, 0)),
    )
    fig_1_4 = plt.arrow(
        point1_arrow[0],
        point1_arrow[1],
        vector_arrow[0],
        vector_arrow[1],
        width=0.3,
        head_width=1.0,
        head_length=1.0,
        fc="g",
        ec="g",
    )
    plt.legend(
        [fig_1_1, fig_1_2, fig_1_3[0], fig_1_4],
        ["Left cones", "Right cones", "Initial poses", "Driving direction"],
        handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)},
        loc="upper right",
        fontsize=20,
    )
    ax_1 = plt.gca()
    ax_1.set_title("Initial data", fontsize=20)
    ax_1.set_aspect(aspect="equal", adjustable="datalim")
    plt.xlabel("x-distance from original car position in m", fontsize=20)
    plt.ylabel("y-distance from original car position in m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Initial data")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 2: Quadratic interpolation --------------------------------------------------------------------------------
    # - Left cones (fig_2_1)
    # - Right cones (fig_2_2)
    # - Initial poses (fig_2_3)
    # - Interpolated trajectory (fig_2_4)
    # - Interpolated left boundary (fig_2_5)
    # - Interpolated right boundary (fig_2_6)
    # - Driving direction arrow (fig_2_7)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure("Quadratic interpolation")
    plt.grid()
    fig_2_1 = plt.scatter(
        cones_left[:, 0],
        cones_left[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_2_2 = plt.scatter(
        cones_right[:, 0],
        cones_right[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_2_3 = plt.scatter(
        initial_poses[:, 0],
        initial_poses[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_2_4 = plt.plot(
        trajectory_interp_cl[:, 0],
        trajectory_interp_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=0.5,
        linestyle="dashed",
        marker="o",
        markersize=3,
        markerfacecolor=normalize_color((0, 0, 0)),
    )
    fig_2_5 = plt.plot(
        bound_left_interp_cl[:, 0],
        bound_left_interp_cl[:, 1],
        color=normalize_color((0, 0, 255)),
        linewidth=0.5,
        linestyle="dashed",
        marker="o",
        markersize=4,
        markerfacecolor=normalize_color((0, 0, 255)),
    )
    fig_2_6 = plt.plot(
        bound_right_interp_cl[:, 0],
        bound_right_interp_cl[:, 1],
        color=normalize_color((230, 240, 0)),
        linewidth=0.5,
        linestyle="dashed",
        marker="o",
        markersize=4,
        markerfacecolor=normalize_color((230, 240, 0)),
    )
    fig_2_7 = plt.arrow(
        point1_arrow[0],
        point1_arrow[1],
        vector_arrow[0],
        vector_arrow[1],
        width=0.3,
        head_width=1.0,
        head_length=1.0,
        fc="g",
        ec="g",
    )
    plt.legend(
        [
            fig_2_1,
            fig_2_4[0],
            fig_2_5[0],
            fig_2_6[0],
            fig_2_7,
        ],
        [
            "Initial data" "Interpolated trajectory",
            "Interpolated left boundary",
            "Interpolated right boundary",
            "Driving direction",
        ],
        handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)},
        loc="upper right",
        fontsize=20,
    )
    ax_2 = plt.gca()
    ax_2.set_title("Quadratic interpolation", fontsize=20)
    ax_2.set_aspect(aspect="equal", adjustable="datalim")
    plt.xlabel("x-distance from original car position in m", fontsize=20)
    plt.ylabel("y-distance from original car position in m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Quadratic interpolation")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 3: BSpline smoothing --------------------------------------------------------------------------------------
    # - Left cones (fig_3_1)
    # - Right cones (fig_3_2)
    # - Initial poses (fig_3_3)
    # - Interpolated trajectory (fig_3_4)
    # - Interpolated left boundary (fig_3_5)
    # - Interpolated right boundary (fig_3_6)
    # - Smoothed trajectory (fig_3_7)
    # - Smoothed left boundary (fig_3_8)
    # - Smoothed right boundary (fig_3_9)
    # - Driving direction arrow (fig_3_10)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure("BSpline smoothing")
    plt.grid()
    fig_3_1 = plt.scatter(
        cones_left[:, 0],
        cones_left[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_3_2 = plt.scatter(
        cones_right[:, 0],
        cones_right[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_3_3 = plt.scatter(
        initial_poses[:, 0],
        initial_poses[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_3_4 = plt.plot(
        trajectory_interp_cl[:, 0],
        trajectory_interp_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=0.3,
        linestyle="dashed",
        marker="o",
        markersize=2,
        markerfacecolor=normalize_color((0, 0, 0)),
    )
    fig_3_5 = plt.plot(
        bound_left_interp_cl[:, 0],
        bound_left_interp_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=0.3,
        linestyle="dashed",
        marker="o",
        markersize=2,
        markerfacecolor=normalize_color((0, 0, 0)),
    )
    fig_3_6 = plt.plot(
        bound_right_interp_cl[:, 0],
        bound_right_interp_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=0.3,
        linestyle="dashed",
        marker="o",
        markersize=2,
        markerfacecolor=normalize_color((0, 0, 0)),
    )
    fig_3_7 = plt.plot(
        trajectory_smoothed_cl[:, 0],
        trajectory_smoothed_cl[:, 1],
        color=normalize_color((0, 255, 0)),
        linewidth=1.5,
        linestyle="solid",
    )
    fig_3_8 = plt.plot(
        bound_left_smoothed_cl[:, 0],
        bound_left_smoothed_cl[:, 1],
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
        linestyle="solid",
    )
    fig_3_9 = plt.plot(
        bound_right_smoothed_cl[:, 0],
        bound_right_smoothed_cl[:, 1],
        color=normalize_color((230, 240, 0)),
        linewidth=2.0,
        linestyle="solid",
    )
    fig_3_10 = plt.arrow(
        point1_arrow[0],
        point1_arrow[1],
        vector_arrow[0],
        vector_arrow[1],
        width=0.3,
        head_width=1.0,
        head_length=1.0,
        fc="g",
        ec="g",
    )
    plt.legend(
        [
            fig_3_1,
            fig_3_4[0],
            fig_3_7[0],
            fig_3_8[0],
            fig_3_9[0],
            fig_3_10,
        ],
        [
            "Initial data",
            "Interpolated data",
            "Smoothed trajectory",
            "Smoothed left boundary",
            "Smoothed right boundary",
            "Driving direction",
        ],
        handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)},
        loc="upper right",
        fontsize=20,
    )
    ax_3 = plt.gca()
    ax_3.set_title("BSpline smoothing", fontsize=20)
    ax_3.set_aspect(aspect="equal", adjustable="datalim")
    plt.xlabel("x-distance from original car position in m", fontsize=20)
    plt.ylabel("y-distance from original car position in m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "BSpline smoothing")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 4: Boundary distance estimation ---------------------------------------------------------------------------
    # - Left cones (fig_4_1)
    # - Right cones (fig_4_2)
    # - Initial poses (fig_4_3)
    # - Smoothed trajectory (fig_4_4)
    # - Smoothed left boundary (fig_4_5)
    # - Smoothed right boundary (fig_4_6)
    # - New left boundary (fig_4_7)
    # - New right boundary (fig_4_8)
    # - Normals to the smoothed trajectory (fig_4_9)
    # - Driving direction arrow (fig_4_10)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure("Boundary distance estimation")
    plt.grid()
    fig_4_1 = plt.scatter(
        cones_left[:, 0],
        cones_left[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_4_2 = plt.scatter(
        cones_right[:, 0],
        cones_right[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_4_3 = plt.scatter(
        initial_poses[:, 0],
        initial_poses[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_4_4 = plt.plot(
        trajectory_smoothed_cl[:, 0],
        trajectory_smoothed_cl[:, 1],
        color=normalize_color((0, 255, 0)),
        linewidth=1.5,
        linestyle="dashed",
        marker="o",
        markersize=4,
        markerfacecolor=normalize_color((0, 255, 0)),
    )
    fig_4_5 = plt.plot(
        bound_left_smoothed_cl[:, 0],
        bound_left_smoothed_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=1.0,
        linestyle="dashed",
    )
    fig_4_6 = plt.plot(
        bound_right_smoothed_cl[:, 0],
        bound_right_smoothed_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=1.0,
        linestyle="dashed",
    )
    fig_4_7 = plt.plot(
        new_bound_left_cl[:, 0],
        new_bound_left_cl[:, 1],
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
        linestyle="solid",
        marker="o",
        markersize=3,
        markerfacecolor=normalize_color((0, 0, 255)),
    )
    fig_4_8 = plt.plot(
        new_bound_right_cl[:, 0],
        new_bound_right_cl[:, 1],
        color=normalize_color((230, 240, 0)),
        linewidth=2.0,
        linestyle="solid",
        marker="o",
        markersize=3,
        markerfacecolor=normalize_color((230, 240, 0)),
    )
    for i in range(prepped_track.shape[0]):
        temp = np.vstack((new_bound_left_cl[i], new_bound_right_cl[i]))
        fig_4_9 = plt.plot(
            temp[:, 0],
            temp[:, 1],
            color=normalize_color((255, 0, 0)),
            linewidth=0.7,
            linestyle="dashed",
        )
    fig_4_10 = plt.arrow(
        point1_arrow[0],
        point1_arrow[1],
        vector_arrow[0],
        vector_arrow[1],
        width=0.3,
        head_width=1.0,
        head_length=1.0,
        fc="g",
        ec="g",
    )
    plt.legend(
        [
            fig_4_1,
            fig_4_4[0],
            fig_4_5[0],
            fig_4_7[0],
            fig_4_8[0],
            fig_4_9[0],
            fig_4_10,
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
        handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)},
        loc="upper right",
        fontsize=20,
    )
    ax_4 = plt.gca()
    ax_4.set_title("Boundary distance estimation", fontsize=20)
    ax_4.set_aspect(aspect="equal", adjustable="datalim")
    plt.xlabel("x-distance from original car position in m", fontsize=20)
    plt.ylabel("y-distance from original car position in m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Boundary distance estimation")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 5: Preprocessed track -------------------------------------------------------------------------------------
    # - Preprocessed trajectory (fig_5_1)
    # - Preprocessed left boundary (fig_5_2)
    # - Preprocessed right boundary (fig_5_3)
    # - Driving direction arrow (fig_5_4)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure("Preprocessed track")
    plt.grid()
    fig_5_1 = plt.plot(
        trajectory_smoothed_cl[:, 0],
        trajectory_smoothed_cl[:, 1],
        color=normalize_color((0, 255, 0)),
        linewidth=2.0,
        linestyle="solid",
    )
    fig_5_2 = plt.plot(
        new_bound_left_cl[:, 0],
        new_bound_left_cl[:, 1],
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
        linestyle="solid",
    )
    fig_5_3 = plt.plot(
        new_bound_right_cl[:, 0],
        new_bound_right_cl[:, 1],
        color=normalize_color((230, 240, 0)),
        linewidth=2.0,
        linestyle="solid",
    )
    fig_5_4 = plt.arrow(
        point1_arrow[0],
        point1_arrow[1],
        vector_arrow[0],
        vector_arrow[1],
        width=0.3,
        head_width=1.0,
        head_length=1.0,
        fc="g",
        ec="g",
    )
    plt.legend(
        [
            fig_5_1[0],
            fig_5_2[0],
            fig_5_3[0],
            fig_5_4,
        ],
        [
            "Preprocessed trajectory",
            "Preprocessed left boundary",
            "Preprocessed right boundary",
            "Driving direction",
        ],
        handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)},
        loc="upper right",
        fontsize=20,
    )
    ax_5 = plt.gca()
    ax_5.set_title("Preprocessed track", fontsize=20)
    ax_5.set_aspect(aspect="equal", adjustable="datalim")
    plt.xlabel("x-distance from original car position in m", fontsize=20)
    plt.ylabel("y-distance from original car position in m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Preprocessed track")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 6: Curvature comparison reference track and preprocessed track --------------------------------------------
    # - Curvature reference track (fig_6_1)
    # - Curvature preprocessed track (fig_6_2)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure("Curvature comparison reference track and preprocessed track")
    plt.grid()
    fig_6_1 = plt.plot(
        s_ref,
        kappa_ref,
        color=normalize_color((255, 0, 0)),
        linewidth=1.5,
        linestyle="solid",
    )
    fig_6_2 = plt.plot(
        s_prepped,
        kappa_prepped,
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
        linestyle="solid",
    )
    plt.legend(
        [
            fig_6_1[0],
            fig_6_2[0],
        ],
        [
            "Curvature reference track",
            "Curvature preprocessed track",
        ],
        loc="upper right",
        fontsize=20,
    )
    ax_6 = plt.gca()
    ax_6.set_title(
        "Curvature comparison reference track and preprocessed track", fontsize=20
    )
    ax_6.set_aspect(aspect="auto", adjustable="datalim")
    plt.xlabel("s-distance along the track in m", fontsize=20)
    plt.ylabel("Curvature in 1/m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Curvature comparison")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 7: Curvature derivative comparison reference track and preprocessed track ---------------------------------
    # - Curvature derivative reference track (fig_7_1)
    # - Curvature derivative preprocessed track (fig_7_2)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure("Curvature derivative comparison reference track and preprocessed track")
    plt.grid()
    fig_7_1 = plt.plot(
        s_ref,
        dkappa_ref,
        color=normalize_color((255, 0, 0)),
        linewidth=1.5,
        linestyle="solid",
    )
    fig_7_2 = plt.plot(
        s_prepped,
        dkappa_prepped,
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
        linestyle="solid",
    )
    plt.legend(
        [
            fig_7_1[0],
            fig_7_2[0],
        ],
        [
            "Curvature derivative reference track",
            "Curvature derivative preprocessed track",
        ],
        loc="upper right",
        fontsize=20,
    )
    ax_7 = plt.gca()
    ax_7.set_title(
        "Curvature derivative comparison reference track and preprocessed track",
        fontsize=20,
    )
    ax_7.set_aspect(aspect="auto", adjustable="datalim")
    plt.xlabel("s-distance along the track in m", fontsize=20)
    plt.ylabel("Curvature derivative in 1/m^2", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Curvature derivative comparison")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    if calc_vel:
        # --------------------------------------------------------------------------------------------------------------
        # FIGURE 8: Velocity profile preprocessed track ----------------------------------------------------------------
        # - Velocity profile preprocessed track (fig_8_1)
        # --------------------------------------------------------------------------------------------------------------
        plt.figure("Velocity profile preprocessed track")
        plt.grid()
        fig_8_1 = plt.plot(
            s_prepped,
            vx_profile_prepped,
            color=normalize_color((255, 0, 0)),
            linewidth=2.0,
            linestyle="solid",
        )
        plt.legend(
            [
                fig_8_1[0],
            ],
            [
                "Velocity profile preprocessed track",
            ],
            loc="upper right",
            fontsize=20,
        )
        ax_8 = plt.gca()
        ax_8.set_title("Velocity profile preprocessed track", fontsize=20)
        ax_8.set_aspect(aspect="auto", adjustable="datalim")
        plt.xlabel("s-distance along the track in m", fontsize=20)
        plt.ylabel("Velocity in m/s", fontsize=20)

        if save_plots:
            save_plot_to_folder(plt, folder_path, "Velocity profile preprocessed track")

        if show_plots:
            plt.show()
        else:
            plt.close()
            plt.clf()

        # --------------------------------------------------------------------------------------------------------------
        # FIGURE 9: Velocity profile preprocessed track 3D -------------------------------------------------------------
        # - Preprocessed trajectory (fig_9_1)
        # - Preprocessed left boundary (fig_9_2)
        # - Preprocessed right boundary (fig_9_3)
        # - Velocity profile preprocessed track in 3D (fig_9_4)
        # --------------------------------------------------------------------------------------------------------------
        fig_9 = plt.figure("Velocity profile of the preprocessed track in 3D")
        ax_9 = fig_9.add_subplot(projection="3d")
        ax_9.set_title("Velocity profile preprocessed track 3D", fontsize=20)
        ax_9.grid()

        # Scale the z-axis such that it does not appear stretched
        fig_9_scale_x = 1.0
        fig_9_scale_y = 1.0
        fig_9_scale_z = 0.3

        # recast get_proj function to use scaling factors for the axes
        ax_9.get_proj = lambda: np.dot(
            Axes3D.get_proj(ax_9),
            np.diag([fig_9_scale_x, fig_9_scale_y, fig_9_scale_z, 1]),
        )

        fig_9_1 = ax_9.plot(
            raceline_prepped_cl[:, 0],
            raceline_prepped_cl[:, 1],
            color=normalize_color((255, 0, 255)),
            linewidth=1.0,
            linestyle="dashed",
        )
        fig_9_2 = ax_9.plot(
            new_bound_left_cl[:, 0],
            new_bound_left_cl[:, 1],
            0.0,
            color=normalize_color((0, 0, 255)),
            linewidth=1.0,
            linestyle="solid",
        )
        fig_9_3 = ax_9.plot(
            new_bound_right_cl[:, 0],
            new_bound_right_cl[:, 1],
            0.0,
            color=normalize_color((230, 240, 0)),
            linewidth=1.0,
            linestyle="solid",
        )
        fig_9_4 = ax_9.plot(
            raceline_prepped_cl[:, 0],
            raceline_prepped_cl[:, 1],
            vx_profile_prepped_cl,
            color=normalize_color((0, 0, 0)),
            linewidth=2.0,
            linestyle="solid",
        )
        ax_9.legend(
            [
                fig_9_1[0],
                fig_9_2[0],
                fig_9_3[0],
                fig_9_4[0],
            ],
            [
                "Preprocessed trajectory",
                "Preprocessed left boundary",
                "Preprocessed right boundary",
                "Velocity profile preprocessed track 3D",
            ],
            fontsize=20,
        )

        ax_9.set_aspect("equalxy")
        ax_9.set_xlabel("x-distance from original car position in m", fontsize=10)
        ax_9.set_ylabel("y-distance from original car position in m", fontsize=10)
        ax_9.set_zlabel("velocity in m/s", fontsize=10)

        # plot vertical lines visualizing acceleration and deceleration zones
        ind_stepsize = int(
            np.round(1.0 / raceline_prepped_cl[1, 0] - raceline_prepped_cl[0, 0])
        )
        if ind_stepsize < 1:
            ind_stepsize = 1

        cur_ind = 0
        no_points_traj_vdc = np.shape(raceline_prepped_cl)[0]

        while cur_ind < no_points_traj_vdc - 1:
            x_tmp = [raceline_prepped_cl[cur_ind, 0], raceline_prepped_cl[cur_ind, 0]]
            y_tmp = [raceline_prepped_cl[cur_ind, 1], raceline_prepped_cl[cur_ind, 1]]
            z_tmp = [
                0.0,
                vx_profile_prepped_cl[cur_ind],
            ]  # plot line with height depending on velocity

            # get proper color for line depending on acceleration
            if ax_profile_prepped_cl[cur_ind] > 0.0:
                col = "g"
            elif ax_profile_prepped_cl[cur_ind] < 0.0:
                col = "r"
            else:
                col = "gray"

            # plot line
            ax_9.plot(x_tmp, y_tmp, z_tmp, color=col)

            # increment index
            cur_ind += ind_stepsize

        if save_plots:
            save_plot_to_folder(
                fig_9, folder_path, "Velocity profile preprocessed track 3D"
            )

        if show_plots:
            plt.show()
        else:
            plt.close()
            plt.clf()

        # --------------------------------------------------------------------------------------------------------------
        # FIGURE 10: Acceleration profile preprocessed track -----------------------------------------------------------
        # - Acceleration profile preprocessed track (fig_10_1)
        # --------------------------------------------------------------------------------------------------------------
        plt.figure("Acceleration profile preprocessed track")
        plt.grid()
        fig_10_1 = plt.plot(
            s_prepped,
            ax_profile_prepped,
            color=normalize_color((255, 0, 0)),
            linewidth=2.0,
            linestyle="solid",
        )
        plt.legend(
            [
                fig_10_1[0],
            ],
            [
                "Acceleration profile preprocessed track",
            ],
            loc="upper right",
            fontsize=20,
        )
        ax_10 = plt.gca()
        ax_10.set_title("Acceleration profile preprocessed track", fontsize=20)
        ax_10.set_aspect(aspect="auto", adjustable="datalim")
        plt.xlabel("s-distance along the track in m", fontsize=20)
        plt.ylabel("Acceleration in m/s^2", fontsize=20)

        if save_plots:
            save_plot_to_folder(
                plt, folder_path, "Acceleration profile preprocessed track"
            )

        if show_plots:
            plt.show()
        else:
            plt.close()
            plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # LINTING ISSUES BUGFIX --------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    fig_2_2 = fig_2_2
    fig_2_3 = fig_2_3

    fig_3_2 = fig_3_2
    fig_3_3 = fig_3_3
    fig_3_5 = fig_3_5
    fig_3_6 = fig_3_6

    fig_4_2 = fig_4_2
    fig_4_3 = fig_4_3
    fig_4_6 = fig_4_6

    # ------------------------------------------------------------------------------------------------------------------


def plot_optimisation(
    cones_left_: np.ndarray,
    cones_right_: np.ndarray,
    initial_poses_: np.ndarray,
    width_veh_real: float,
    width_veh_opt: float,
    prepped_track_: np.ndarray,
    normvec_norm_prepped_: np.ndarray,
    kappa_prepped_: np.ndarray,
    dkappa_prepped_: np.ndarray,
    s_prepped_: np.ndarray,
    bound_left_: np.ndarray,
    bound_right_: np.ndarray,
    opt_track_: np.ndarray,
    normvec_norm_opt_: np.ndarray,
    kappa_opt_: np.ndarray,
    dkappa_opt_: np.ndarray,
    s_opt_: np.ndarray,
    plot_options: dict,
    vx_profile_prepped: np.ndarray = None,
    ax_profile_prepped: np.ndarray = None,
    vx_profile_opt: np.ndarray = None,
    ax_profile_opt: np.ndarray = None,
    raceline_opt_: np.ndarray = None,
):
    """
    author:
    Kwinten Mortier

    .. description::
    This function plots the optimisation results.

    .. inputs::
    :param cones_left:              array containing the left track cones [x, y] (unclosed!).
    :type cones_left:               np.ndarray
    :param cones_right:             array containing the right track cones [x, y] (unclosed!).
    :type cones_right:              np.ndarray
    :param initial_poses:           array containing the poses of the trajectory from pathplanning [x, y] (unclosed!).
    :type initial_poses:            np.ndarray
    :param width_veh_real:          real width of the vehicle.
    :type width_veh_real:           float
    :param width_veh_opt:           width of the vehicle used for the optimisation.
    :type width_veh_opt:            float
    :param prepped_track:           array containing the preprocessed track, i.e. a preprocessed racing line and the
                                    according track widths to the left and to the right [x, y, w_tr_left, w_tr_right]
                                    (unit is meter, unclosed!)
    :type prepped_track:            np.ndarray
    :param normvec_norm_prepped:    normal vectors for every point of the preprocessed track [x, y] (unit is 1, unclosed!).
    :type normvec_norm_prepped:     np.ndarray
    :param kappa_prepped:           curvature for every point of the preprocessed track [x, y] (unit is 1/m, unclosed!).
    :type kappa_prepped:            np.ndarray
    :param dkappa_prepped:          derivative of curvature for every point of the preprocessed track [x, y]
                                    (unit is 1/m^2, unclosed!)
    :type dkappa_prepped:           np.ndarray
    :param s_prepped:               s-distance along the preprocessed track in m.
    :type s_prepped:                np.ndarray
    :param bound_left:              array containing the left boundary of the preprocessed track [x, y] (unclosed!).
    :type bound_left:               np.ndarray
    :param bound_right:             array containing the right boundary of the preprocessed track [x, y] (unclosed!).
    :type bound_right:              np.ndarray
    :param opt_track:               array containing the optimised track, i.e. an optimised racing line and the
                                    according track widths to the left and to the right [x, y, w_tr_left, w_tr_right]
                                    (unit is meter, unclosed!)
    :type opt_track:                np.ndarray
    :param normvec_norm_opt:        normal vectors for every point of the optimised track [x, y] (unit is 1, unclosed!).
    :type normvec_norm_opt:         np.ndarray
    :param kappa_opt:               curvature for every point of the optimised track [x, y] (unit is 1/m, unclosed!).
    :type kappa_opt:                np.ndarray
    :param dkappa_opt:              derivative of curvature for every point of the optimised track [x, y]
                                    (unit is 1/m^2, unclosed!)
    :type dkappa_opt:               np.ndarray
    :param s_opt:                   s-distance along the optimised track in m.
    :type s_opt:                    np.ndarray
    :param plot_options:            dictionary containing the plot options to determine which plots to show.
    :type plot_options:             dict
    :param vx_profile_prepped:      velocity profile of the preprocessed track in m/s.
    :type vx_profile_prepped:       np.ndarray
    :param ax_profile_prepped:      acceleration profile of the preprocessed track in m/s^2.
    :type ax_profile_prepped:       np.ndarray
    :param vx_profile_opt:          velocity profile of the optimised track in m/s.
    :type vx_profile_opt:           np.ndarray
    :param ax_profile_opt:          acceleration profile of the optimised track in m/s^2.
    :type ax_profile_opt:           np.ndarray
    :param raceline_opt:            array containing the raceline of the optimised track.
    :type raceline_opt:             np.ndarray

    .. notes::
    Track inputs are unclosed! Track inputs must however be closable in the current form!

    OPTIMISATION PLOTS:
    - FIG 1: Preprocessed preprocessed track with vehicle visualisation and boundaries.
    - FIG 2: Optimised track with vehicle visualisation and boundaries.
    - FIG 3: Comparison between the preprocessed trajectory and the optimised trajectory.
    - FIG 4: Curvature comparison between the preprocessed track and the optimised track.
    - FIG 5: Curvature derivative comparison between the preprocessed track and the optimised track.

    OPTIONAL OPTIMISATION PLOTS FOR VELOCITY AND ACCELERATION PROFILES (if calc_vel is True):
    - FIG 6: Velocity profile of the optimised track.
    - FIG 7: Velocity profile of the optimised track in 3D.
    - FIG 8: Acceleration profile of the optimised track.
    - FIG 9: Velocity profile comparison between the preprocessed track and the optimised track.
    - FIG 10: Acceleration profile comparison between the preprocessed track and the optimised track.
    """

    # ------------------------------------------------------------------------------------------------------------------
    # COPY INPUTS ------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    cones_left = np.copy(cones_left_)
    cones_right = np.copy(cones_right_)
    initial_poses = np.copy(initial_poses_)
    prepped_track = np.copy(prepped_track_)
    normvec_norm_prepped = np.copy(normvec_norm_prepped_)
    kappa_prepped = np.copy(kappa_prepped_)
    dkappa_prepped = np.copy(dkappa_prepped_)
    s_prepped = np.copy(s_prepped_)
    bound_left = np.copy(bound_left_)
    bound_right = np.copy(bound_right_)
    opt_track = np.copy(opt_track_)
    normvec_norm_opt = np.copy(normvec_norm_opt_)
    kappa_opt = np.copy(kappa_opt_)
    dkappa_opt = np.copy(dkappa_opt_)
    s_opt = np.copy(s_opt_)
    raceline_opt = np.copy(raceline_opt_)

    # ------------------------------------------------------------------------------------------------------------------
    # CHECKING INPUTS --------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    if (
        (cones_left[0] == cones_left[-1]).all()
        or (cones_right[0] == cones_right[-1]).all()
        or (initial_poses[0] == initial_poses[-1]).all()
        or (prepped_track[0] == prepped_track[-1]).all()
        or (opt_track[0] == opt_track[-1]).all()
    ):
        raise RuntimeError("Initial data and tracks must be unclosed!")

    if (
        cones_left.shape[1] != 2
        or cones_right.shape[1] != 2
        or initial_poses.shape[1] != 2
        or bound_left.shape[1] != 2
        or bound_right.shape[1] != 2
    ):
        raise RuntimeError("Initial data and boudnaries must have the shape [x, y]!")

    if prepped_track.shape[1] != 4 or opt_track.shape[1] != 4:
        raise RuntimeError("Tracks must have the shape [x, y, w_tr_left, w_tr_right]!")

    if (
        (kappa_prepped is not None and kappa_prepped[0] == kappa_prepped[-1].all())
        or (kappa_opt is not None and kappa_opt[0] == kappa_opt[-1].all())
        or (
            dkappa_prepped is not None and dkappa_prepped[0] == dkappa_prepped[-1].all()
        )
        or (dkappa_opt is not None and dkappa_opt[0] == dkappa_opt[-1].all())
    ):
        raise RuntimeError("Curvature and curvature derivative data must be unclosed!")

    # ------------------------------------------------------------------------------------------------------------------
    # DEFINING VARIABLES -----------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    initial_poses_cl = np.vstack((initial_poses, initial_poses[0]))
    prepped_track_cl = np.vstack((prepped_track, prepped_track[0]))
    opt_track_cl = np.vstack((opt_track, opt_track[0]))

    # Closed boundaries
    bound_left_cl = np.vstack((bound_left, bound_left[0]))
    bound_right_cl = np.vstack((bound_right, bound_right[0]))

    # Calculate real and virtual vehicle boundaries for the reference track
    veh_bound_left_ref_real = (
        prepped_track[:, :2] - normvec_norm_prepped * width_veh_real / 2
    )
    veh_bound_right_ref_real = (
        prepped_track[:, :2] + normvec_norm_prepped * width_veh_real / 2
    )

    veh_bound_left_ref_virt = (
        prepped_track[:, :2] - normvec_norm_prepped * width_veh_opt / 2
    )
    veh_bound_right_ref_virt = (
        prepped_track[:, :2] + normvec_norm_prepped * width_veh_opt / 2
    )

    veh_bound_left_ref_real_cl = np.vstack(
        (veh_bound_left_ref_real, veh_bound_left_ref_real[0])
    )
    veh_bound_right_ref_real_cl = np.vstack(
        (veh_bound_right_ref_real, veh_bound_right_ref_real[0])
    )
    veh_bound_left_ref_virt_cl = np.vstack(
        (veh_bound_left_ref_virt, veh_bound_left_ref_virt[0])
    )
    veh_bound_right_ref_virt_cl = np.vstack(
        (veh_bound_right_ref_virt, veh_bound_right_ref_virt[0])
    )

    # Calculate real and virtual vehicle boundaries for the optimised track
    veh_bound_left_opt_real = opt_track[:, :2] - normvec_norm_opt * width_veh_real / 2
    veh_bound_right_opt_real = opt_track[:, :2] + normvec_norm_opt * width_veh_real / 2

    veh_bound_left_opt_virt = opt_track[:, :2] - normvec_norm_opt * width_veh_opt / 2
    veh_bound_right_opt_virt = opt_track[:, :2] + normvec_norm_opt * width_veh_opt / 2

    veh_bound_left_opt_real_cl = np.vstack(
        (veh_bound_left_opt_real, veh_bound_left_opt_real[0])
    )
    veh_bound_right_opt_real_cl = np.vstack(
        (veh_bound_right_opt_real, veh_bound_right_opt_real[0])
    )
    veh_bound_left_opt_virt_cl = np.vstack(
        (veh_bound_left_opt_virt, veh_bound_left_opt_virt[0])
    )
    veh_bound_right_opt_virt_cl = np.vstack(
        (veh_bound_right_opt_virt, veh_bound_right_opt_virt[0])
    )

    # Arrow variables for the legend (driving direction)
    point1_arrow = initial_poses[0]
    point2_arrow = initial_poses[2]
    vector_arrow = point2_arrow - point1_arrow

    # Determine the plot options
    opt_method = plot_options["opt_method"]
    calc_vel = plot_options["calc_vel"]
    show_plots = plot_options["show_plots"]
    save_plots = plot_options["save_plots"]
    folder_path = plot_options["folder_path"]

    # TODO: DEZE LIJST VERDER AANVULLEN VOOR CORRECTE TITELS EN LEGENDA'S
    if opt_method == "shortest_path":
        fig_2_name = "Shortest path optimised track"
        fig_2_title = "Shortest path optimised track"
        fig_2_10_legend = "Shortest path trajectory"
        fig_3_title = "Preprocessed track vs shortest path optimised trajectory"
    elif opt_method == "iqp_mincurv":
        fig_2_name = "Minimum curvature optimised track"
        fig_2_title = "Minimum curvature optimised track"
        fig_2_10_legend = "Minimum curvature trajectory"
        fig_3_title = (
            "Preprocessed trajectory vs minimum curvature optimised trajectory"
        )

    # Closed variables for 3D plot
    if calc_vel:
        vx_profile_opt_cl = np.hstack((vx_profile_opt, vx_profile_opt[0]))
        ax_profile_opt_cl = np.hstack((ax_profile_opt, ax_profile_opt[0]))
        raceline_opt_cl = np.vstack((raceline_opt, raceline_opt[0]))

    ####################################################################################################################
    # PLOTTING FIGURES
    # - FIGURE 1: Preprocessed track with vehicle visualisation and boundaries.
    # - FIGURE 2: Optimised track with vehicle visualisation and boundaries.
    # - FIGURE 3: Comparison between the preprocessed trajectory and the optimised trajectory.
    # - FIGURE 4: Curvature comparison between the preprocessed track and the optimised track.
    # - FIGURE 5: Curvature derivative comparison between the preprocessed track and the optimised track.
    #
    # OPTIONAL PLOTTING FIGURES FOR VELOCITY AND ACCELERATION PROFILES (if calc_vel is True)
    # - FIGURE 6: Velocity profile optimised track.
    # - FIGURE 7: Velocity profile optimised track 3D.
    # - FIGURE 8: Acceleration profile optimised track.
    # - Figure 9: Velocity profile comparison between the preprocessed track and the optimised track.
    # - Figure 10: Acceleration profile comparison between the preprocessed track and the optimised track.
    ####################################################################################################################

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 1: Preprocessed track with vehicle visualisation and boundaries -------------------------------------------
    # - Left cones (fig_1_1)
    # - Right cones (fig_1_2)
    # - Initial poses (fig_1_3)
    # - Left boundary (fig_1_4)
    # - Right boundary (fig_1_5)
    # - Left virtual vehicle boundary (fig_1_6)
    # - Right virtual vehicle boundary (fig_1_7)
    # - Left real vehicle boundary (fig_1_8)
    # - Right real vehicle boundary (fig_1_9)
    # - Preprocessed trajectory (fig_1_10)
    # - Driving direction arrow (fig_1_11)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure("Preprocessed reference track")
    plt.grid()
    fig_1_1 = plt.scatter(
        cones_left[:, 0],
        cones_left[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_1_2 = plt.scatter(
        cones_right[:, 0],
        cones_right[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_1_3 = plt.plot(
        initial_poses_cl[:, 0],
        initial_poses_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=1,
        linestyle="dashed",
        marker="o",
        markersize=3,
        markerfacecolor=normalize_color((0, 0, 0)),
    )
    fig_1_4 = plt.plot(
        bound_left_cl[:, 0],
        bound_left_cl[:, 1],
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
    )
    fig_1_5 = plt.plot(
        bound_right_cl[:, 0],
        bound_right_cl[:, 1],
        color=normalize_color((230, 240, 0)),
        linewidth=2.0,
    )
    fig_1_6 = plt.plot(
        veh_bound_left_ref_virt_cl[:, 0],
        veh_bound_left_ref_virt_cl[:, 1],
        color=normalize_color((255, 0, 255)),
        linewidth=1.0,
        linestyle="dashed",
    )
    fig_1_7 = plt.plot(
        veh_bound_right_ref_virt_cl[:, 0],
        veh_bound_right_ref_virt_cl[:, 1],
        color=normalize_color((255, 0, 255)),
        linewidth=1.0,
        linestyle="dashed",
    )
    fig_1_8 = plt.plot(
        veh_bound_left_ref_real_cl[:, 0],
        veh_bound_left_ref_real_cl[:, 1],
        color=normalize_color((0, 255, 255)),
        linewidth=1.0,
    )
    fig_1_9 = plt.plot(
        veh_bound_right_ref_real_cl[:, 0],
        veh_bound_right_ref_real_cl[:, 1],
        color=normalize_color((0, 255, 255)),
        linewidth=1.0,
    )
    fig_1_10 = plt.plot(
        prepped_track_cl[:, 0],
        prepped_track_cl[:, 1],
        color=normalize_color((255, 0, 0)),
        linewidth=2.0,
    )
    fig_1_11 = plt.arrow(
        point1_arrow[0],
        point1_arrow[1],
        vector_arrow[0],
        vector_arrow[1],
        width=0.3,
        head_width=1.0,
        head_length=1.0,
        fc="g",
        ec="g",
    )
    plt.legend(
        [
            fig_1_1,
            fig_1_3[0],
            fig_1_4[0],
            fig_1_5[0],
            fig_1_6[0],
            fig_1_8[0],
            fig_1_10[0],
            fig_1_11,
        ],
        [
            "Cones",
            "Initial poses",
            "Left boundary",
            "Right boundary",
            "Virtual vehicle boundary",
            "Real vehicle boundary",
            "Preprocessed trajectory",
            "Driving direction",
        ],
        handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)},
        loc="upper right",
        fontsize=20,
    )
    ax_1 = plt.gca()
    ax_1.set_title("Preprocessed reference track", fontsize=20)
    ax_1.set_aspect(aspect="equal", adjustable="datalim")
    plt.xlabel("x-distance from original car position in m", fontsize=20)
    plt.ylabel("y-distance from original car position in m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Preprocessed reference track")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 2: Optimised track with vehicle visualisation and boundaries ----------------------------------------------
    # - Left cones (fig_2_1)
    # - Right cones (fig_2_2)
    # - Initial poses (fig_2_3)
    # - Left boundary (fig_2_4)
    # - Right boundary (fig_2_5)
    # - Left virtual vehicle boundary (fig_2_6)
    # - Right virtual vehicle boundary (fig_2_7)
    # - Left real vehicle boundary (fig_2_8)
    # - Right real vehicle boundary (fig_2_9)
    # - Optimised trajectory (fig_2_10)
    # - Driving direction arrow (fig_2_11)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure(fig_2_name)
    plt.grid()
    fig_2_1 = plt.scatter(
        cones_left[:, 0],
        cones_left[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_2_2 = plt.scatter(
        cones_right[:, 0],
        cones_right[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_2_3 = plt.plot(
        initial_poses_cl[:, 0],
        initial_poses_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=1,
        linestyle="dashed",
        marker="o",
        markersize=3,
        markerfacecolor=normalize_color((0, 0, 0)),
    )
    fig_2_4 = plt.plot(
        bound_left_cl[:, 0],
        bound_left_cl[:, 1],
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
    )
    fig_2_5 = plt.plot(
        bound_right_cl[:, 0],
        bound_right_cl[:, 1],
        color=normalize_color((230, 240, 0)),
        linewidth=2.0,
    )
    fig_2_6 = plt.plot(
        veh_bound_left_opt_virt_cl[:, 0],
        veh_bound_left_opt_virt_cl[:, 1],
        color=normalize_color((255, 0, 255)),
        linewidth=1.0,
        linestyle="dashed",
    )
    fig_2_7 = plt.plot(
        veh_bound_right_opt_virt_cl[:, 0],
        veh_bound_right_opt_virt_cl[:, 1],
        color=normalize_color((255, 0, 255)),
        linewidth=1.0,
        linestyle="dashed",
    )
    fig_2_8 = plt.plot(
        veh_bound_left_opt_real_cl[:, 0],
        veh_bound_left_opt_real_cl[:, 1],
        color=normalize_color((0, 255, 255)),
        linewidth=1.0,
    )
    fig_2_9 = plt.plot(
        veh_bound_right_opt_real_cl[:, 0],
        veh_bound_right_opt_real_cl[:, 1],
        color=normalize_color((0, 255, 255)),
        linewidth=1.0,
    )
    fig_2_10 = plt.plot(
        opt_track_cl[:, 0],
        opt_track_cl[:, 1],
        color=normalize_color((255, 0, 0)),
        linewidth=2.0,
    )
    fig_2_11 = plt.arrow(
        point1_arrow[0],
        point1_arrow[1],
        vector_arrow[0],
        vector_arrow[1],
        width=0.3,
        head_width=1.0,
        head_length=1.0,
        fc="g",
        ec="g",
    )
    plt.legend(
        [
            fig_2_1,
            fig_2_3[0],
            fig_2_4[0],
            fig_2_5[0],
            fig_2_6[0],
            fig_2_8[0],
            fig_2_10[0],
            fig_2_11,
        ],
        [
            "Cones",
            "Initial poses",
            "Left boundary",
            "Right boundary",
            "Virtual vehicle boundary",
            "Real vehicle boundary",
            fig_2_10_legend,
            "Driving direction",
        ],
        handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)},
        loc="upper right",
        fontsize=20,
    )
    ax_2 = plt.gca()
    ax_2.set_title(fig_2_title, fontsize=20)
    ax_2.set_aspect(aspect="equal", adjustable="datalim")
    plt.xlabel("x-distance from original car position in m", fontsize=20)
    plt.ylabel("y-distance from original car position in m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, fig_2_name)

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 3: Comparison between the preprocessed track and the optimised track --------------------------------------
    # - Left cones (fig_3_1)
    # - Right cones (fig_3_2)
    # - Initial poses (fig_3_3)
    # - Left boundary (fig_3_4)
    # - Right boundary (fig_3_5)
    # - Preprocessed trajectory (fig_3_6)
    # - Optimised trajectory (fig_3_7)
    # - Driving direction arrow (fig_3_8)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure("Reference vs. optimised track")
    plt.grid()
    fig_3_1 = plt.scatter(
        cones_left[:, 0],
        cones_left[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_3_2 = plt.scatter(
        cones_right[:, 0],
        cones_right[:, 1],
        color=normalize_color((255, 0, 0)),
        marker="X",
        s=25,
    )
    fig_3_3 = plt.plot(
        initial_poses_cl[:, 0],
        initial_poses_cl[:, 1],
        color=normalize_color((0, 0, 0)),
        linewidth=1,
        linestyle="dashed",
        marker="o",
        markersize=3,
        markerfacecolor=normalize_color((0, 0, 0)),
    )
    fig_3_4 = plt.plot(
        bound_left_cl[:, 0],
        bound_left_cl[:, 1],
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
        linestyle="solid",
    )
    fig_3_5 = plt.plot(
        bound_right_cl[:, 0],
        bound_right_cl[:, 1],
        color=normalize_color((230, 240, 0)),
        linewidth=2.0,
        linestyle="solid",
    )
    fig_3_6 = plt.plot(
        prepped_track_cl[:, 0],
        prepped_track_cl[:, 1],
        color=normalize_color((255, 0, 0)),
        linewidth=2.0,
        linestyle="dashed",
    )
    fig_3_7 = plt.plot(
        opt_track_cl[:, 0],
        opt_track_cl[:, 1],
        color=normalize_color((0, 255, 0)),
        linewidth=2.0,
        linestyle="solid",
    )
    fig_3_8 = plt.arrow(
        point1_arrow[0],
        point1_arrow[1],
        vector_arrow[0],
        vector_arrow[1],
        width=0.3,
        head_width=1.0,
        head_length=1.0,
        fc="g",
        ec="g",
    )
    plt.legend(
        [
            fig_3_1,
            fig_3_3[0],
            fig_3_4[0],
            fig_3_5[0],
            fig_3_6[0],
            fig_3_7[0],
            fig_3_8,
        ],
        [
            "Cones",
            "Initial poses",
            "Left boundary",
            "Right boundary",
            "Preprocessed trajectory",
            "Optimised trajectory",
            "Driving direction",
        ],
        handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow)},
        loc="upper right",
        fontsize=20,
    )
    ax_3 = plt.gca()
    ax_3.set_title(fig_3_title, fontsize=20)
    ax_3.set_aspect(aspect="equal", adjustable="datalim")
    plt.xlabel("x-distance from original car position in m", fontsize=20)
    plt.ylabel("y-distance from original car position in m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Reference vs. optimised track")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 4: Curvature comparison between the preprocessed track and the optimised track ----------------------------
    # - Curvature reference track (fig_4_1)
    # - Curvature optimised track (fig_4_2)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure(
        "Curvature comparison between the reference track and the optimised track"
    )
    plt.grid()
    fig_4_1 = plt.plot(
        s_prepped,
        kappa_prepped,
        color=normalize_color((255, 0, 0)),
        linewidth=1.5,
        linestyle="solid",
    )
    fig_4_2 = plt.plot(
        s_opt,
        kappa_opt,
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
        linestyle="solid",
    )
    plt.legend(
        [
            fig_4_1[0],
            fig_4_2[0],
        ],
        [
            "Curvature preprocessed track",
            "Curvature optimised track",
        ],
        loc="upper right",
        fontsize=20,
    )
    ax_4 = plt.gca()
    ax_4.set_title(
        "Curvature comparison between the reference track and the optimised track",
        fontsize=20,
    )
    ax_4.set_aspect(aspect="auto", adjustable="datalim")
    plt.xlabel("s-distance along the track in m", fontsize=20)
    plt.ylabel("curvature in rad/m", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Curvature comparison")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # FIGURE 5: Curvature derivative comparison between the preprocessed track and the optimised track -----------------
    # - Curvature derivative preprocessed track (fig_5_1)
    # - Curvature derivative optimised track (fig_5_2)
    # ------------------------------------------------------------------------------------------------------------------
    plt.figure(
        "Curvature derivative comparison between the preprocessed track and the optimised track"
    )
    plt.grid()
    fig_5_1 = plt.plot(
        s_prepped,
        dkappa_prepped,
        color=normalize_color((255, 0, 0)),
        linewidth=1.5,
        linestyle="solid",
    )
    fig_5_2 = plt.plot(
        s_opt,
        dkappa_opt,
        color=normalize_color((0, 0, 255)),
        linewidth=2.0,
        linestyle="solid",
    )
    plt.legend(
        [
            fig_5_1[0],
            fig_5_2[0],
        ],
        [
            "Curvature derivative preprocessed track",
            "Curvature derivative optimised track",
        ],
        loc="upper right",
        fontsize=20,
    )
    ax_5 = plt.gca()
    ax_5.set_title(
        "Curvature derivative comparison between the reference track and the optimised track",
        fontsize=20,
    )
    ax_5.set_aspect(aspect="auto", adjustable="datalim")
    plt.xlabel("s-distance along the track in m", fontsize=20)
    plt.ylabel("curvature derivative in rad/m^2", fontsize=20)

    if save_plots:
        save_plot_to_folder(plt, folder_path, "Curvature derivative comparison")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()

    if calc_vel:
        # --------------------------------------------------------------------------------------------------------------
        # FIGURE 6: Velocity profile of the optimised track ------------------------------------------------------------
        # - Velocity profile optimised track (fig_6_1)
        # --------------------------------------------------------------------------------------------------------------
        plt.figure("Velocity profile of the optimised track")
        plt.grid()
        fig_6_1 = plt.plot(
            s_opt,
            vx_profile_opt,
            color=normalize_color((255, 0, 0)),
            linewidth=2.0,
            linestyle="solid",
        )
        plt.legend(
            [
                fig_6_1[0],
            ],
            [
                "Velocity profile optimised track",
            ],
            loc="upper right",
            fontsize=20,
        )
        ax_6 = plt.gca()
        ax_6.set_title("Velocity profile of the optimised track", fontsize=20)
        ax_6.set_aspect(aspect="auto", adjustable="datalim")
        plt.xlabel("s-distance along the track in m", fontsize=20)
        plt.ylabel("velocity in m/s", fontsize=20)

        if save_plots:
            save_plot_to_folder(plt, folder_path, "Velocity profile optimised track")

        if show_plots:
            plt.show()
        else:
            plt.close()
            plt.clf()

        # --------------------------------------------------------------------------------------------------------------
        # FIGURE 7: Velocity profile of the optimised track in 3D ------------------------------------------------------
        # - Optimised trajectory (fig_7_1)
        # - Left track boundary (fig_7_2)
        # - Right track boundary (fig_7_3)
        # - Velocity profile optimised track in 3D (fig_7_4)
        # --------------------------------------------------------------------------------------------------------------
        fig_7 = plt.figure("Velocity profile of the optimised track in 3D")
        ax_7 = fig_7.add_subplot(projection="3d")
        ax_7.set_title("Velocity profile of the optimised track in 3D", fontsize=20)
        ax_7.grid()

        # Scale the z-axis such that it does not appear stretched
        fig_7_scale_x = 1.0
        fig_7_scale_y = 1.0
        fig_7_scale_z = 0.3

        # Recast get_proj function to use scaling factors for the axes
        ax_7.get_proj = lambda: np.dot(
            Axes3D.get_proj(ax_7),
            np.diag([fig_7_scale_x, fig_7_scale_y, fig_7_scale_z, 1]),
        )

        fig_7_1 = ax_7.plot(
            raceline_opt_cl[:, 0],
            raceline_opt_cl[:, 1],
            color=normalize_color((255, 0, 255)),
            linewidth=1.0,
            linestyle="dashed",
        )
        fig_7_2 = ax_7.plot(
            bound_left_cl[:, 0],
            bound_left_cl[:, 1],
            0.0,
            color=normalize_color((0, 0, 255)),
            linewidth=1.0,
            linestyle="solid",
        )
        fig_7_3 = ax_7.plot(
            bound_right_cl[:, 0],
            bound_right_cl[:, 1],
            0.0,
            color=normalize_color((230, 240, 0)),
            linewidth=1.0,
            linestyle="solid",
        )
        fig_7_4 = ax_7.plot(
            raceline_opt_cl[:, 0],
            raceline_opt_cl[:, 1],
            vx_profile_opt_cl,
            color=normalize_color((0, 0, 0)),
            linewidth=2.0,
            linestyle="solid",
        )
        ax_7.legend(
            [
                fig_7_1[0],
                fig_7_2[0],
                fig_7_3[0],
                fig_7_4[0],
            ],
            [
                "Optimised trajectory",
                "Left boundary",
                "Right boundary",
                "Velocity profile optimised track 3D",
            ],
            fontsize=20,
        )

        ax_7.set_aspect("equalxy")
        ax_7.set_xlabel("x-distance from original car position in m", fontsize=20)
        ax_7.set_ylabel("y-distance from original car position in m", fontsize=20)
        ax_7.set_zlabel("velocity in m/s", fontsize=20)

        # plot vertical lines visualizing acceleration and deceleration zones
        ind_stepsize = int(
            np.round(1.0 / raceline_opt_cl[1, 0] - raceline_opt_cl[0, 0])
        )
        if ind_stepsize < 1:
            ind_stepsize = 1

        cur_ind = 0
        no_points_traj_vdc = np.shape(raceline_opt_cl)[0]

        while cur_ind < no_points_traj_vdc - 1:
            x_tmp = [raceline_opt_cl[cur_ind, 0], raceline_opt_cl[cur_ind, 0]]
            y_tmp = [raceline_opt_cl[cur_ind, 1], raceline_opt_cl[cur_ind, 1]]
            z_tmp = [
                0.0,
                vx_profile_opt_cl[cur_ind],
            ]  # plot line with height depending on velocity

            # get proper color for line depending on acceleration
            if ax_profile_opt_cl[cur_ind] > 0.0:
                col = "g"
            elif ax_profile_opt_cl[cur_ind] < 0.0:
                col = "r"
            else:
                col = "gray"

            # plot line
            ax_7.plot(x_tmp, y_tmp, z_tmp, color=col)

            # increment index
            cur_ind += ind_stepsize

        if save_plots:
            save_plot_to_folder(
                fig_7, folder_path, "Velocity profile optimised track 3D"
            )

        if show_plots:
            plt.show()
        else:
            plt.close()
            plt.clf()

        # --------------------------------------------------------------------------------------------------------------
        # FIGURE 8: Acceleration profile of the optimised track --------------------------------------------------------
        # - Acceleration profile optimised track (fig_8_1)
        # --------------------------------------------------------------------------------------------------------------
        plt.figure("Acceleration profile of the optimised track")
        plt.grid()
        fig_8_1 = plt.plot(
            s_opt,
            ax_profile_opt,
            color=normalize_color((255, 0, 0)),
            linewidth=2.0,
            linestyle="solid",
        )
        plt.legend(
            [
                fig_8_1[0],
            ],
            [
                "Acceleration profile optimised track",
            ],
            loc="upper right",
            fontsize=20,
        )
        ax_8 = plt.gca()
        ax_8.set_title("Acceleration profile of the optimised track", fontsize=20)
        ax_8.set_aspect(aspect="auto", adjustable="datalim")
        plt.xlabel("s-distance along the track in m", fontsize=20)
        plt.ylabel("acceleration in m/s^2", fontsize=20)

        if save_plots:
            save_plot_to_folder(
                plt, folder_path, "Acceleration profile optimised track"
            )

        if show_plots:
            plt.show()
        else:
            plt.close()
            plt.clf()

        # --------------------------------------------------------------------------------------------------------------
        # FIGURE 9: Velocity profile comparison between the preprocessed track and the optimised track -----------------
        # - Velocity profile preprocessed track (fig_9_1)
        # - Velocity profile optimised track (fig_9_2)
        # --------------------------------------------------------------------------------------------------------------
        plt.figure(
            "Velocity profile comparison between the reference track and the optimised track"
        )
        plt.grid()
        fig_9_1 = plt.plot(
            s_prepped,
            vx_profile_prepped,
            color=normalize_color((255, 0, 0)),
            linewidth=1.5,
            linestyle="solid",
        )
        fig_9_2 = plt.plot(
            s_opt,
            vx_profile_opt,
            color=normalize_color((0, 0, 255)),
            linewidth=2.0,
            linestyle="solid",
        )
        plt.legend(
            [
                fig_9_1[0],
                fig_9_2[0],
            ],
            [
                "Velocity profile preprocessed track",
                "Velocity profile optimised track",
            ],
            loc="upper right",
            fontsize=20,
        )
        ax_9 = plt.gca()
        ax_9.set_title(
            "Velocity profile comparison between the reference track and the optimised track",
            fontsize=20,
        )
        ax_9.set_aspect(aspect="auto", adjustable="datalim")
        plt.xlabel("s-distance along the track in m", fontsize=20)
        plt.ylabel("velocity in m/s", fontsize=20)

        if save_plots:
            save_plot_to_folder(plt, folder_path, "Velocity profile comparison")

        if show_plots:
            plt.show()
        else:
            plt.close()
            plt.clf()

        # --------------------------------------------------------------------------------------------------------------
        # FIGURE 10: Acceleration profile comparison between the preprocessed track and the optimised track ------------
        # - Acceleration profile preprocessed track (fig_10_1)
        # - Acceleration profile optimised track (fig_10_2)
        # --------------------------------------------------------------------------------------------------------------
        plt.figure(
            "Acceleration profile comparison between the reference track and the optimised track"
        )
        plt.grid()
        fig_10_1 = plt.plot(
            s_prepped,
            ax_profile_prepped,
            color=normalize_color((255, 0, 0)),
            linewidth=1.5,
            linestyle="solid",
        )
        fig_10_2 = plt.plot(
            s_opt,
            ax_profile_opt,
            color=normalize_color((0, 0, 255)),
            linewidth=2.0,
            linestyle="solid",
        )
        plt.legend(
            [
                fig_10_1[0],
                fig_10_2[0],
            ],
            [
                "Acceleration profile preprocessed track",
                "Acceleration profile optimised track",
            ],
            loc="upper right",
            fontsize=20,
        )
        ax_10 = plt.gca()
        ax_10.set_title(
            "Acceleration profile comparison between the reference track and the optimised track",
            fontsize=20,
        )
        ax_10.set_aspect(aspect="auto", adjustable="datalim")
        plt.xlabel("s-distance along the track in m", fontsize=20)
        plt.ylabel("acceleration in m/s^2", fontsize=20)

        if save_plots:
            save_plot_to_folder(plt, folder_path, "Acceleration profile comparison")

        if show_plots:
            plt.show()
        else:
            plt.close()
            plt.clf()

    # ------------------------------------------------------------------------------------------------------------------
    # LINTING ISSUES BUGFIX --------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    fig_1_2 = fig_1_2
    fig_1_7 = fig_1_7
    fig_1_9 = fig_1_9

    fig_2_2 = fig_2_2
    fig_2_7 = fig_2_7
    fig_2_9 = fig_2_9

    fig_3_2 = fig_3_2

    # ------------------------------------------------------------------------------------------------------------------
