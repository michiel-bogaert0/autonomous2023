import copy
from typing import Callable

import casadi
import numpy as np
from optimal_control_gen.integrator import euler, rk4
from utils_gen.common_utils import Timer


class Ocp:
    def __init__(
        self,
        nx: int,
        nu: int,
        N: int,
        curve,
        T: float = None,
        F: Callable = None,
        silent=False,
        show_execution_time=True,
        store_intermediate=False,
        threads=1,
    ):
        """optimal control abstraction

        Args:
            nx (int): state size
            nu (int): action size
            N (int): number of control intervals
            T (float): time horizon, Defaults to None
            f (callable): dynamics x_dot=f(x, u) which returns the derivative of the state x_dot
            F (callable): discrete dynamics x_new=F(x,u)
            silent (bool, optional): print optimisation steps. Defaults to False.
            store_intermediate (bool, optional): store intermediate solutions of the optimisation problem. Defaults to False.
            integrator (str, optional): the integrator used in the discretization. Supports Runge-Kutta and Euler. Defaults to 'euler'.
        """
        self.nx = nx
        self.nu = nu
        self.N = N
        self.T = T
        self.F = F
        self.dt = T / N

        self.opti = casadi.Opti()

        self.centerline = curve
        self.der_centerline = curve.derivative(o=1)

        self.X = self.opti.variable(self.nx, N + 1)
        # self.X = casadi.repmat([1e1, 1e1, 1e-1, 1, 1e2, 1], 1, self.N+1) * self.opti.variable(self.nx, N + 1)
        self.U = self.opti.variable(self.nu, N)
        self.x0 = self.opti.parameter(self.nx)
        self.u_prev = self.opti.parameter(self.nu)

        # Soften constraints
        self.Sc = self.opti.variable(1, N + 1)
        self.sc = casadi.SX.sym("sc", 1)

        self.params = []  # additional parameters

        # symbolic params to define cost functions
        self.x = casadi.SX.sym("symbolic_x", self.nx)
        self.u = casadi.SX.sym("symbolic_u", self.nu)
        self.u_delta = casadi.SX.sym("symbolic_u_prev", self.nu)
        self.x_reference = casadi.SX.sym("symbolic_x_control_", self.nx)

        # Point on curve
        self.point_curve = casadi.SX.sym("point_curve", 2)
        self.der_curve = casadi.SX.sym("der_curve", 2)

        self.slopes_inner = self.opti.parameter(1, N + 1)
        self.intercepts_inner = self.opti.parameter(1, N + 1)
        self.slopes_outer = self.opti.parameter(1, N + 1)
        self.intercepts_outer = self.opti.parameter(1, N + 1)
        # self.b = self.opti.variable(1)
        # self.c = self.opti.variable(1)
        # self.d = self.opti.variable(1)

        self._set_continuity(threads)

        self.cost_fun = None
        self.cost = {"run": 0, "total": 0}

        self.set_solver(silent, print_time=show_execution_time)

        self.X_sol_intermediate = []
        self.U_sol_intermediate = []
        self.store_intermediate_flag = False
        if store_intermediate:
            self.store_intermediate()

        self.timer = Timer(verbose=False)

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))
        return result

    def __getattr__(self, name):
        return getattr(self.opti, name)

    def discretize(self, f, DT, M, integrator="rk4"):
        x = casadi.SX.sym("x", self.nx)
        u = casadi.SX.sym("u", self.nu)
        # dt = casis..;

        if integrator == "rk4":
            x_new = rk4(f, x, u, DT, M)
        elif integrator == "euler":
            # x_new = x + f(x, u)*DT
            x_new = euler(f, x, u, DT, M)
        else:
            raise Exception("integrator not recognized")

        F = casadi.Function("F", [x, u], [x_new], ["x", "u"], ["x_new"])

        return F

    def _set_continuity(self, threads: int):
        # self.opti.subject_to(self.X[2:, 0] == self.x0[2:]) # doing this somehow causes crash issue
        # self.opti.subject_to(self.X[0, 0] == self.x0[0])
        # self.opti.subject_to(self.X[2, 0] == self.x0[2])
        # self.opti.subject_to(self.X[3, 0] == 0)
        # self.opti.subject_to(self.X[4, 0] == self.x0[4])
        self.opti.subject_to(self.X[5, 0] == self.x0[5])
        self.opti.subject_to(self.X[5, self.N] == 1)
        # self.opti.subject_to(self.X[0, 0] == self.X[0, self.N])
        # self.opti.subject_to(self.X[1, 0] == self.X[1, self.N])

        if threads == 1:
            for i in range(self.N):
                x_next = self.F(self.X[:, i], self.U[:, i])  # hier self.DT_sym meegeven

                if isinstance(x_next, np.ndarray):
                    x_next = casadi.vcat(x_next)  # convert numpy array to casadi vector

                # self.opti.subject_to(self.X[0, i + 1] == x_next[0])
                # self.opti.subject_to(self.X[1, i + 1] == x_next[1])
                # self.opti.subject_to(casadi.fmod(self.X[2, i + 1], 2*casadi.pi) == casadi.fmod(x_next[2], 2*casadi.pi))
                # self.opti.subject_to(casadi.fmod(self.X[3, i + 1], 2*casadi.pi) == casadi.fmod(x_next[3], 2*casadi.pi))
                # self.opti.subject_to((self.X[3, i + 1] % 2*casadi.pi) == (x_next[3]% 2*casadi.pi))
                # self.opti.subject_to(self.X[4, i + 1] == x_next[4])
                # self.opti.subject_to(self.X[5, i + 1] == x_next[5])
                self.opti.subject_to(self.X[:, i + 1] == x_next)
        else:
            X_next = self.F.map(self.N, "thread", threads)(self.X[:, :-1], self.U)
            self.opti.subject_to(self.X[:, 1:] == X_next)

    def eval_cost(self, X, U, Sc):
        # Function not tested
        assert X.shape[0] == self.nx
        # N = X.shape[1] - 1

        L_run = 0  # cost over the horizon
        for i in range(self.N + 1):
            if i == 0:
                L_run += self.cost_fun(
                    X[:, i],
                    U[:, i],
                    (U[:, i] - self.u_prev),
                    self.centerline(X[5, i]).T,
                    self.der_centerline(X[5, i]).T,
                    Sc[i],
                )
            elif i == self.N:
                L_run += self.cost_fun(
                    X[:, i],
                    0,
                    0,
                    self.centerline(X[5, i]).T,
                    self.der_centerline(X[5, i]).T,
                    Sc[i],
                )
            else:
                L_run += self.cost_fun(
                    X[:, i],
                    U[:, i],
                    (U[:, i] - U[:, i - 1]),
                    self.centerline(X[5, i]).T,
                    self.der_centerline(X[5, i]).T,
                    Sc[i],
                )

        return casadi.sum2(L_run)

    def set_cost(
        self,
        cost_fun=None,
    ):
        if cost_fun is not None:
            self.cost_fun = cost_fun
            L_run = 0  # cost over the horizon
            for i in range(self.N + 1):
                if i == 0:
                    L_run += cost_fun(
                        self.X[:, i],
                        self.U[:, i],
                        (self.U[:, i] - self.u_prev),
                        self.centerline(self.X[5, i]).T,
                        self.der_centerline(self.X[5, i]).T,
                        self.Sc[i],
                    )
                elif i == self.N:
                    L_run += cost_fun(
                        self.X[:, i],
                        0,
                        0,
                        self.centerline(self.X[5, i]).T,
                        self.der_centerline(self.X[5, i]).T,
                        self.Sc[i],
                    )
                else:
                    L_run += cost_fun(
                        self.X[:, i],
                        self.U[:, i],
                        (self.U[:, i] - self.U[:, i - 1]),
                        self.centerline(self.X[5, i]).T,
                        self.der_centerline(self.X[5, i]).T,
                        self.Sc[i],
                    )

            self.cost["run"] = L_run

        self.cost["total"] = self.cost["run"]
        self.opti.minimize(self.cost["total"])

    @property
    def running_cost(self):
        return self.cost["run"]

    @running_cost.setter
    def running_cost(self, symbolic_cost):
        cost_fun = casadi.Function(
            "cost_fun",
            [
                self.x,
                self.u,
                self.u_delta,
                self.point_curve,
                self.der_curve,
                self.sc,
            ],
            [symbolic_cost],
        )
        self.set_cost(cost_fun=cost_fun)

    def set_solver(self, silent, print_time=1, solver="ipopt"):
        """set the solver to be used for the trajectory optimisation

        Args:
            silent(bool): print to the output console
            solver (str, optional): the solver to be used. Defaults to 'ipopt'.

        Raises:
            Exception: if the chosen solver is not recognized
        """
        if solver == "ipopt":
            p_opts, s_opts = self.solver_settings(
                silent, print_time=print_time, solver="ipopt"
            )
            self.opti.solver("ipopt", p_opts, s_opts)

        else:
            raise Exception("solver option not recognized")

    def solver_settings(self, silent=False, print_time=1, solver="ipopt"):
        assert solver == "ipopt"

        if silent:
            print_level = 0
        else:
            print_level = 5

        p_opts = {
            "print_time": print_time,  # print information about execution time (if True, also stores it in sol stats)
        }
        s_opts = {
            "max_iter": 1000,
            "print_level": print_level,
            "sb": "yes",
            # "nlp_scaling_method": "none",
            "constr_viol_tol": 1e-1,
            "print_user_options": "yes",
            # "acceptable_constr_viol_tol": 1e-1,
            # "bound_relax_factor": 10,
        }

        return p_opts, s_opts

    def solve(
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
        show_exception=True,
    ):
        """solve the optimal control problem for the initial state and goal state

        Args:
            state (array): initial state
            goal_state (array): the goal state
            control_states (array): the control states
            X0 (array, optional): the initial state sequence guess. Defaults to None.
            U0 (array, optional): the initial action sequence guess. Defaults to None.
            lin_interpol (bool, optional): build a linear interpolation between the init and goal state. Defaults to False.

        Returns:
            [tuple]: U_sol [nu, N], X_sol [nx, N], info
        """
        self.opti.set_value(self.x0, state)

        self.centerline = curve
        self.der_centerline = curve.derivative(o=1)

        # self.opti.set_initial(self.a, a)
        # self.opti.set_initial(self.b, b)
        # self.opti.set_initial(self.c, c)
        # self.opti.set_initial(self.d, d)
        self.opti.set_value(self.slopes_inner, a)
        self.opti.set_value(self.intercepts_inner, b)
        self.opti.set_value(self.slopes_outer, c)
        self.opti.set_value(self.intercepts_outer, d)

        self.opti.set_value(self.u_prev, u_prev)

        if X0 is not None:
            self.opti.set_initial(self.X, X0)
        else:
            self.opti.set_initial(self.X, np.zeros((self.nx, self.N + 1)))

        if U0 is not None:
            self.opti.set_initial(self.U, U0)
        else:
            self.opti.set_initial(self.U, np.zeros((self.nu, self.N)))

        self.opti.set_initial(self.Sc, np.zeros((1, self.N + 1)) * 1e-1)

        try:
            with self.timer:
                self.sol = self.opti.solve()

            self.U_sol = np.array(
                self.sol.value(self.U)
            )  # array in case of scalar control
            self.X_sol = self.sol.value(self.X)
            success = self.sol.stats()["success"]
            if "t_proc_total" in self.sol.stats().keys():
                time = self.sol.stats()["t_proc_total"]
            else:
                time = self.timer.time
                # time = (
                #     self.sol.stats()["t_proc_nlp_f"]
                #     + self.sol.stats()["t_proc_nlp_g"]
                #     + self.sol.stats()["t_proc_nlp_grad"]
                #     + self.sol.stats()["t_proc_nlp_grad_f"]
                #     + self.sol.stats()["t_proc_nlp_hess_l"]
                #     + self.sol.stats()["t_proc_nlp_jac_g"]
                #     + self.sol.stats()["t_wall_nlp_f"]
                #     + self.sol.stats()["t_wall_nlp_g"]
                #     + self.sol.stats()["t_wall_nlp_grad"]
                #     + self.sol.stats()["t_wall_nlp_grad_f"]
                #     + self.sol.stats()["t_wall_nlp_hess_l"]
                #     + self.sol.stats()["t_wall_nlp_jac_g"]
                # )
        except Exception as e:
            if show_exception:
                print(e)
            self.sol = self.opti.debug
            self.U_sol = self.sol.value(self.U)
            self.X_sol = self.sol.value(self.X)
            success = False
            time = self.timer.time

        info = {
            "success": success,
            "cost": self.sol.value(self.opti.f),
            "time": time,
            "time_hess": self.sol.stats()["t_proc_nlp_hess_l"],
            "iter_count": self.sol.stats()["iter_count"],
        }

        return (
            self.U_sol.reshape((-1, self.N)),
            self.X_sol.reshape((-1, self.N + 1)),
            info,
        )
        # return self.U_sol.reshape((self.N, -1)), self.X_sol.reshape((self.N+1, -1)), info

    def store_intermediate(self):
        if self.store_intermediate_flag is True:
            return
        else:
            self.store_intermediate_flag = True

        def store(i):
            x = self.opti.debug.value(self.X).reshape((-1, self.N + 1))
            u = np.array(self.opti.debug.value(self.U)).reshape((-1, self.N))

            self.X_sol_intermediate.append(x)
            self.U_sol_intermediate.append(u)

        self.opti.callback(store)
