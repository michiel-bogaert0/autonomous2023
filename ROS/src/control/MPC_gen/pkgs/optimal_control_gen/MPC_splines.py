import casadi as cd
import numpy as np
from casadi import casadi
from optimal_control_gen.ocp import Ocp
from utils_gen.common_utils import Timer


class MPC_splines:
    """
    Abstraction of MPC, can be called to run MPC with splines. Uses casadi
    formulation to run the dynamic optimisation.

    Kwargs:
        the casadi opti abstraction and associated variables and parameters that form
        the problem formulation
    """

    def __init__(self, ocp: Ocp, silent=True, store_intermediate=False):
        self.ocp = ocp

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

        self.store_intermediate = store_intermediate
        if self.store_intermediate:
            ocp.store_intermediate()
            self._X_sol_intermediate = ocp.X_sol_intermediate
            self._U_sol_intermediate = ocp.U_sol_intermediate
            # self.X_sol_list_intermediate = []
            # self.U_sol_list_intermediate = []
            # self.store_intermediate()

        # self.print_intermediate()

    def __call__(
        self,
        state,
        curve,
        a,
        b,
        c,
        d,
        u_prev,
        X0=None,
        U0=None,
        warm_start=True,
        **solver_kwargs,
    ):
        """Solve the optimal control problem for the given initial state and return the first action.
        If an initial guess is provided, the warm start flag is ignored.

        Args:
            state (array): initial state
            ref_state (array): goal state
            control_states (array): control states
            X0 (array, optional): initial guess of the state sequence over the horizon [nx, N+1]. Defaults to None.
            U0 (array, optional): initial guess of the action sequence over the whole horizon [nu, N]. Defaults to None.
            warm_start (bool, optional): shift the previous solution one timestep to warm start the current problem. Defaults to True.
            lin_interpol (bool, optional): warm start solution, if X0 is given, only interpolate first call

        Returns:
            array: the first control from the control sequence solution
        """

        if warm_start:
            if self.nr_iters == 0:
                if self.X_init_guess is not None:
                    X0 = self.X_init_guess
            else:
                if (X0 is None) and (self.X_sol is not None):
                    # Some small remark to this code: the commented out code is what I used originally, but I think it is better
                    # to not shift the previous solution, as we are working in baselink frame. However, I could be wrong in my reasoning.

                    # shift previous solution to warm start current optim, use array in slice, i.e. [-1], to preserve dim
                    # X0 = np.concatenate(
                    #     (self.X_sol[:, 1:], self.X_sol[:, [-1]]), axis=1
                    # )
                    X0 = self.X_sol

            if (U0 is None) and (self.U_sol is not None):
                if len(self.U_sol.shape) > 1:
                    # U0 = np.concatenate(
                    #     (self.U_sol[:, 1:], self.U_sol[:, [-1]]), axis=1
                    # )
                    U0 = self.U_sol
                # else:
                #     U0 = np.append(self.U_sol[1:], self.U_sol[-1])

        with self.timer:
            self.U_sol, self.X_sol, info = self.ocp.solve(
                state, curve, a, b, c, d, u_prev, X0=X0, U0=U0, **solver_kwargs
            )

        self.X_sol_list.append(self.X_sol)
        self.U_sol_list.append(self.U_sol)

        self.computation_time.append(info["time"])

        info["X_sol"] = self.X_sol
        info["U_sol"] = self.U_sol

        u = np.array(self.U_sol[:, 0])
        if len(u.shape) == 2:
            u = u.squeeze()  # discard singleton dimensions

        self.X_trajectory.append(np.array(state).squeeze())
        self.U_trajectory.append(u)

        self.nr_iters += 1

        return u, info

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

        I have never used this function
        """

        import matplotlib.pyplot as plt

        # print(self.opti.g.shape)
        # print(self.opti.g[0])

        x = cd.vertcat(cd.vec(self.X_sol), self.U_sol.T)

        if L is None:
            L = self.ocp.f
        H, J = cd.hessian(L, x)

        plt.spy(H.sparsity())
        plt.show()

        Jg = cd.jacobian(self.ocp.g, x)
        plt.spy(Jg.sparsity())
        plt.show()

    def showConvergence(self):
        """
        I have never used this function
        """
        import matplotlib.pyplot as plt

        inf_pr = self.ocp.debug.stats()["iterations"]["inf_pr"]
        inf_du = self.ocp.debug.stats()["iterations"]["inf_du"]

        fig, ax = plt.subplots()

        ax.plot(np.arange(len(inf_pr)), inf_pr, label="inf_pr")
        ax.plot(np.arange(len(inf_du)), inf_du, label="inf_du")

        ax.legend(loc="upper right")
        ax.set_yscale("log")

        plt.show()

    def evaluate_objective(self, x, x_ref):
        """
        I have never used this function
        """
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
