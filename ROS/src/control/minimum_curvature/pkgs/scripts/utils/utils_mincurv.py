import math
import time

import numpy as np
import quadprog
from scipy.interpolate import interp1d, splev, splprep


def generate_interpolated_points(path):
    # Linear interpolation between center points to add more points for BSpline smoothing
    distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
    distance = np.insert(distance, 0, 0) / distance[-1]

    alpha = np.linspace(0, 1, len(path) * 3)
    interpolator = interp1d(distance, path, kind="linear", axis=0)
    path = interpolator(alpha)

    return path


def B_spline_smoothing(path):
    per = 1  # BSPline periodicity, 0 = not periodic, 1 = periodic

    # Smooth path with BSpline interpolation
    path = path.T  # Transpose to get correct shape for BSpline, splprep expects (2, N)
    w = np.array(
        [1] * len(path[0])
    )  # Weights for BSpline (Here, same weight for every point)

    tck, u = splprep(path, w=w, s=5, per=per)  # Calculate BSpline
    smoothed_path = np.array(
        splev(u, tck)
    ).T  # Evaluate BSpline and transpose back to (N, 2)

    return smoothed_path


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
    template_M = (
        np.array(  # current point               | next point              | bounds
            [
                [
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                ],  # a_0i                                                  = {x,y}_i
                [
                    1,
                    1,
                    1,
                    1,
                    0,
                    0,
                    0,
                    0,
                ],  # a_0i + a_1i +  a_2i +  a_3i                           = {x,y}_i+1
                [
                    0,
                    1,
                    2,
                    3,
                    0,
                    -1,
                    0,
                    0,
                ],  # _      a_1i + 2a_2i + 3a_3i      - a_1i+1             = 0
                [0, 0, 2, 6, 0, 0, -2, 0],
            ]
        )
    )  # _             2a_2i + 6a_3i               - 2a_2i+1   = 0

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
                        the right and to the left [x, y, w_tr_right, w_tr_left] (unit is meter, must be unclosed!)
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
    dev_max_right = reftrack[:, 2] - w_veh / 2
    dev_max_left = reftrack[:, 3] - w_veh / 2

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

    # calculate maximum curvature error
    curv_error_max = np.amax(np.abs(curv_sol_lin - curv_orig_lin))

    return alpha_mincurv, curv_error_max
