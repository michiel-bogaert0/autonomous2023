import casadi as cd
import numpy as np
import rospy
from casadi import casadi
from environments.kinematic_car import KinematicCar
from utils.common_utils import Timer

# TODO: modify the shape of the return array, squeeze return state


class MPC:
    """
    Abstraction of MPC, can be called to run trajectory optimisation. Uses casadi
    formulation to run the dynamic optimisation.

    Kwargs:
        the casadi opti abstraction and associated variables and parameters that form
        the problem formulation
    """

    def __init__(
        self,
        nx,
        nu,
        car: KinematicCar,
        T,
        terminal_constraint,
    ):
        self.nx = nx
        self.nu = nu
        self.opti = casadi.Opti()

        self.N_horizon = round(
            T / car.dt
        )  # will we do pred horizon based on steps or time?

        self.X = self.opti.variable(
            self.nx, self.N_horizon + 1
        )  # states variables to be optimised
        self.U = self.opti.variable(
            self.nu, self.N_horizon
        )  # control variable to be optimised
        self.x0 = self.opti.parameter(self.nx)  # initial state

        self.x_ref = self.opti.parameter(self.nx)  # reference state

        self.F = car.F  # dynamics function

        # initial state and continuity constraint
        self._set_continuity(1)

        if terminal_constraint:
            self.opti.subject_to(
                self.X[:, self.N_horizon] == self.x_ref[:, self.N_horizon]
            )

        # symbolic params to define cost functions
        self.x_sym = casadi.SX.sym("symbolic_x", self.nx)
        self.u_sym = casadi.SX.sym("symbolic_u", self.nu)
        self.x_ref_sym = casadi.SX.sym("symbolic_x_ref", self.nx)

        self.cost_fun = None
        self.terminal_cost_fun = None
        self.cost = {"run": 0, "terminal": 0, "total": 0}

        p_opts = {"print_time": 0}
        s_opts = {"max_iter": 5, "print_level": 0, "sb": "yes"}
        self.opti.solver("ipopt", p_opts, s_opts)

        self.sol = None
        self.X_sol = None
        self.U_sol = None

        self.X_init_guess = None

        self.X_sol_list = []
        self.U_sol_list = []
        # self.results = []

        self.X_trajectory = []
        self.U_trajectory = []

        self.computation_time = []
        self.timer = Timer(verbose=False)

        self.nr_iters = 0

    def __call__(self, state, timestep):
        """Solve the optimal control problem for the given initial state and return the first action.
        If an initial guess is provided, the warm start flag is ignored.

        Args:
            state (array): initial state
            ref_state (array): goal state
            X0 (array, optional): initial guess of the state sequence over the horizon [nx, N+1]. Defaults to None.
            U0 (array, optional): initial guess of the action sequence over the whole horizon [nu, N]. Defaults to None.
            lin_interpol (bool, optional): warm start solution, if X0 is given, only interpolate first call

        Returns:
            array: the first control from the control sequence solution
        """

        self.opti.set_value(self.x0, state)

        x_ref_now = self.x_ref[:, timestep]  # x_ref should be updated based on path?

        x_guess = x_ref_now
        u_guess = self.U[:, 0]  # take previous input?

        self.opti.set_initial(self.X, x_guess)
        self.opti.set_initial(self.U, u_guess)

        try:
            self.sol = self.opti.solve()
            self.U_sol = self.sol.value(self.U).reshape((-1, self.N_horizon))
            self.X_sol = self.sol.value(self.X).reshape((-1, self.N_horizon + 1))
            success = self.sol.stats()["success"]
        except Exception as e:
            self.sol = self.opti.debug
            self.U_sol = self.sol.value(self.U).reshape((-1, self.N_horizon))
            self.X_sol = self.sol.value(self.X).reshape((-1, self.N_horizon + 1))
            success = False
            rospy.logerr(f"MPC caught an error: {e}")

        info = {
            "success": success,
            "cost": self.sol.value(self.opti.f),
        }
        return (self.U_sol, self.X_sol, info)

    def _set_continuity(self, threads: int):
        self.opti.subject_to(self.X[:, 0] == self.x0)

        if threads == 1:
            for i in range(self.N):
                x_next = self.F(self.X[:, i], self.U[:, i])

                if isinstance(x_next, np.ndarray):
                    x_next = casadi.vcat(x_next)  # convert numpy array to casadi vector

                self.opti.subject_to(self.X[:, i + 1] == x_next)
        else:
            X_next = self.F.map(self.N, "thread", threads)(self.X[:, :-1], self.U)
            self.opti.subject_to(self.X[:, 1:] == X_next)

    def reset(self, init_state=None):
        self.X_sol = None
        self.U_sol = None

        self.nr_iters = 0

        if hasattr(self, "_X_sol_intermediate"):
            self.ocp.X_sol_intermediate = []
            self.ocp.U_sol_intermediate = []

        self.X_trajectory = []
        self.U_trajectory = []

        if init_state is not None:
            self.X_trajectory.append(np.array(init_state).squeeze())

    def analyseDeriv(self, L=None):
        """Show the pattern of the jacobian and hessian of the cost functions and of the jacobian of the constraints.
        If no cost L is supplied, the cost in the opti formulation is used.
        """

        import matplotlib.pyplot as plt

        # print(self.opti.g.shape)
        # print(self.opti.g[0])

        x = cd.vertcat(cd.vec(self.X), self.U.T)

        if L is None:
            L = self.opti.f
        H, J = cd.hessian(L, x)

        plt.spy(H.sparsity())
        plt.show()

        Jg = cd.jacobian(self.opti.g, x)
        plt.spy(Jg.sparsity())
        plt.show()

    def showConvergence(self):
        import matplotlib.pyplot as plt

        inf_pr = self.opti.debug.stats()["iterations"]["inf_pr"]
        inf_du = self.opti.debug.stats()["iterations"]["inf_du"]

        fig, ax = plt.subplots()

        ax.plot(np.arange(len(inf_pr)), inf_pr, label="inf_pr")
        ax.plot(np.arange(len(inf_du)), inf_du, label="inf_du")

        ax.legend(loc="upper right")
        ax.set_yscale("log")

        plt.show()

    def evaluate_objective(self, x, x_ref):
        H, J = cd.hessian(self.opti.f, self.X)
        jac_fun = cd.Function("jac_fun", [self.X, self.x_ref], [J])

        jac = jac_fun(x, x_ref)

        return jac

    @property
    def X_sol_intermediate(self):
        self._X_sol_intermediate = self.ocp.X_sol_intermediate
        return self._X_sol_intermediate

    @property
    def U_sol_intermediate(self):
        self._U_sol_intermediate = self.ocp.U_sol_intermediate
        return self._U_sol_intermediate

    # def get_intermediate(self):

    #     x = self.opti.debug.value(self.X).reshape((-1, self.horizon+1))
    #     u = self.opti.debug.value(self.U).reshape((-1, self.horizon))

    #     return x, u

    # def store_intermediate(self):

    #     def store(i):
    #         x, u = self.get_intermediate()
    #         self.intermediate_sol_state.append(x)
    #         self.intermediate_sol_act.append(u)

    #     self.opti.callback(store)

    # def print_intermediate(self):

    #     # H, J = cd.hessian(self.opti.f, self.X)
    #     # jac_fun = cd.Function('jac_fun', [self.X, self.x_ref], [J])

    #     # def calculate_values(x):
    #     #     return jac_fun(x, np.array([0, 0]))

    #     def show_intermediate_sol(i):

    #         x = self.opti.debug.value(self.X).reshape((-1, self.horizon+1))
    #         self.results.append({'iter': i, 'x': x})

    #         # jacobian = calculate_values(x)
    #         jacobian = self.evaluate_objective(x, np.array([0, 0]))
    #         print(f"iter {i}: {x[:, -1]} -> {jacobian[:, -1]}")

    #     self.opti.callback(show_intermediate_sol)
