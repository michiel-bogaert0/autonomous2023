import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from utils_gen.plotting_utils import normalize_color, save_plot_to_folder


def plot_velocity(
    raceline_opt_,
    bound_left_,
    bound_right_,
    vx_profile_opt_,
    ax_profile_opt_,
    show_plots=False,
    save_plots=False,
    folder_path="",
):
    raceline_opt = np.copy(raceline_opt_)
    bound_left = np.copy(bound_left_)
    bound_right = np.copy(bound_right_)
    vx_profile_opt = np.copy(vx_profile_opt_)
    ax_profile_opt = np.copy(ax_profile_opt_)

    # Close raceline
    raceline_opt_cl = np.vstack((raceline_opt, raceline_opt[0, :]))
    bound_left_cl = np.vstack((bound_left, bound_left[0, :]))
    bound_right_cl = np.vstack((bound_right, bound_right[0, :]))

    # Close velocity profile
    vx_profile_opt_cl = np.hstack((vx_profile_opt, vx_profile_opt[0]))
    ax_profile_opt_cl = np.hstack((ax_profile_opt, ax_profile_opt[0]))

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
    ax_7.set_xlabel("x-distance from original car position in m", fontsize=5)
    ax_7.set_ylabel("y-distance from original car position in m", fontsize=5)
    ax_7.set_zlabel("velocity in m/s", fontsize=5)

    # plot vertical lines visualizing acceleration and deceleration zones
    ind_stepsize = int(np.round(1.0 / raceline_opt_cl[1, 0] - raceline_opt_cl[0, 0]))
    if ind_stepsize < 1:
        ind_stepsize = 1
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
        save_plot_to_folder(fig_7, folder_path, "Velocity profile optimised track 3D")

    if show_plots:
        plt.show()
    else:
        plt.close()
        plt.clf()
