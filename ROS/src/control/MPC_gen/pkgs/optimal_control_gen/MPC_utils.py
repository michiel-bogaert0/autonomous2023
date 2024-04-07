import casadi as cd
import numpy as np
from tqdm import tqdm
from utils.common_utils import get_nr_args


def to_cas(func):
    nr_args = get_nr_args(func)

    if nr_args == 1:

        def wrapper(x):
            x_new = func(x)
            return cd.vertcat(*x_new)

        return wrapper
    elif nr_args == 2:

        def wrapper(x, x_ref):
            x_new = func(x, x_ref)
            return cd.vertcat(*x_new)

        return wrapper
    else:
        raise Exception("unknown number of arguments")


def quadratic_cost(Q_weights, R_weights, obs_fun=lambda x: x):
    """Construct a quadratic cost function for the weights on the state Q_weights
    and the weights on the input R_weights.

    Args:
        Q_weights (1D array): weights on the state Q = diag(Q_weights), x.T*Q*x
        R_weights (1D array): weights on the input R = diag(R_weights), u.T*R*u
        obs_fun (callable, optional): call x_new=obs_fun(x) before calculating the cost: x_new.T*Q*x_new . Defaults to lambdax:x.

    Returns:
        callable: cost_fun = x.T*Q*x + u.T*R*u
    """

    Q = np.diag(Q_weights)
    R = np.diag(R_weights)

    def cost_fun(x, u, x_ref):
        x = obs_fun(x)
        x_ref = obs_fun(x_ref)
        return (x - x_ref).T @ Q @ (x - x_ref) + u.T @ R @ u

    return cost_fun


def terminal_quadratic_cost(P_weights, obs_fun=lambda x: x):
    P = np.diag(P_weights)

    def terminal_cost_fun(x, x_ref):
        x = obs_fun(x)
        x_ref = obs_fun(x_ref)
        return (x - x_ref).T @ P @ (x - x_ref)

    return terminal_cost_fun


def check_control(
    controller, state_space, goal_state=None, goal_state_space=None, nr_samples=100
):
    samples = {"success": [], "fail": []}

    for _ in tqdm(range(nr_samples)):
        init_state = np.random.uniform(low=state_space.low, high=state_space.high)

        if goal_state_space is not None:
            goal_state = np.random.uniform(
                low=goal_state_space.low, high=goal_state_space.high
            )

        # _, _, info = ocp.solve(init_state, goal_state, show_exception=False, **solver_kwargs)
        status = controller(init_state, goal_state)

        if status:
            samples["success"].append(init_state)
        else:
            # print(f"Solution at {init_state} failed")
            samples["fail"].append(init_state)

    success_rate = len(samples["success"]) / nr_samples * 100
    print(f"controller was {success_rate}% successful")

    return samples


def sample_ellipsoid():
    pass


def has_converged_box(
    state, goal_state=None, obs=lambda x: x, lb=None, ub=None, slack=None
):
    assert ((lb is not None) and (ub is not None)) or (
        (slack is not None) and (goal_state is not None)
    )

    if (goal_state is not None) and (slack is not None):
        lb = np.array(goal_state) - np.array(slack)
        ub = np.array(goal_state) + np.array(slack)

    return np.logical_and(
        lb < obs(state),
        obs(state) < ub,
    ).all()


def has_converged_2norm(
    state, goal_state, obs=lambda x: x, weight_convergence=None, tolerance=2e-1
):
    weight_convergence = np.ones(np.shape(obs(goal_state)))
    return (
        np.linalg.norm((obs(goal_state) - obs(state)) * weight_convergence) < tolerance
    )


def trace_constraints(ocp, dependency):
    """The constraints from the ocp are extracted if they depend on the expression
    dependency (should be defined in function of ocp.X, ocp.U). The constraints are compared
    to the corresponding upper and lower bound, ubg and lbg
    """
    g_fun = cd.Function("g", [ocp.X, ocp.U], [ocp.opti.g])
    X_s = cd.SX.sym("X", *ocp.X.shape)
    U_s = cd.SX.sym("U", *ocp.U.shape)
    g_s = g_fun(X_s, U_s)
    dep_fun = cd.Function("dependency", [ocp.X, ocp.U], [dependency])

    X_dep = dep_fun(X_s, U_s)
    xdep = cd.vertcat(cd.vec(X_dep))
    dep_bool = [cd.depends_on(g_s_i, xdep) for g_s_i in cd.vertsplit(g_s)]

    dep_bool_constraints = dep_bool[
        (ocp.nx * (ocp.N + 1)) :
    ]  # skip continuity constraints
    ind_arr = np.arange((ocp.nx * (ocp.N + 1)), g_s.shape[0])[dep_bool_constraints]
    g_fun_red = cd.Function("g_fun_red", [X_dep], [g_s[ind_arr]])

    return lambda x: ocp.opti.lbg[ind_arr] < g_fun_red(x) < ocp.opti.ubg[ind_arr]


def trace_constraints_fun(ocp, *dependency):
    """The constraints from the ocp are extracted if they depend on the expression
    dependency (should be defined in function of ocp.X, ocp.U). The constraints are compared
    to the corresponding upper and lower bound, ubg and lbg
    """
    g_fun = cd.Function("g", [ocp.X, ocp.U], [ocp.opti.g])
    X_s = cd.SX.sym("X", *ocp.X.shape)
    U_s = cd.SX.sym("U", *ocp.U.shape)
    g_s = g_fun(X_s, U_s)
    dep_fun = cd.Function("dependency", [ocp.X, ocp.U], [*dependency])

    X_dep = dep_fun(X_s, U_s)
    if isinstance(X_dep, (list, tuple)):
        xdep = cd.vertcat(*[cd.vec(xdep_i) for xdep_i in X_dep])
    else:
        xdep = cd.vertcat(cd.vec(X_dep))

    dep_bool = [cd.depends_on(g_s_i, xdep) for g_s_i in cd.vertsplit(g_s)]

    dep_bool_constraints = dep_bool[
        (ocp.nx * (ocp.N + 1)) :
    ]  # skip continuity constraints
    ind_arr = np.arange((ocp.nx * (ocp.N + 1)), g_s.shape[0])[dep_bool_constraints]
    if isinstance(X_dep, (list, tuple)):
        g_fun_red = cd.Function("g_fun_red", [*X_dep], [g_s[ind_arr]])
    else:
        g_fun_red = cd.Function("g_fun_red", [X_dep], [g_s[ind_arr]])

    return g_fun_red, ind_arr
