import casadi as cd
import numpy as np
from matplotlib import pyplot as plt
from utils.spline import BSpline, BSplineBasis


class Bspline:
    def __init__(self, kk, degree, Ts=1, N=100):
        knots = np.concatenate([np.ones(degree) * kk[0], kk, np.ones(degree) * kk[-1]])
        self.base = BSplineBasis(knots, degree)

        self.Ts = Ts
        self.tau = np.linspace(0, Ts, N)

        self.degree = degree

        n = len(kk)  # nr knots, including 2*k duplicate knots at the ends
        self.K = n + degree - 1  # nr control points

        self.N = N
        self.arclen_fun = None

    def plot_basis(self):
        plt.plot(cd.densify(self.base(np.linspace(0, 1))))

    def gen_curve(self, x):
        curve = BSpline(self.base, x)
        return curve

    def evaluate_spline(self, x, tau=None):
        """evaluate at tau with control points x"""
        curve = self.gen_curve(x)
        if tau is None:
            tau = self.tau
        return curve(tau)

    def get_flat_output(self, coeff, order, N=None, tau=None):
        if (tau is None) and (N is not None):
            tau = np.linspace(0, self.Ts, int(N))
        elif (tau is None) and (N is None):
            tau = self.tau

        curve = self.gen_curve(coeff)
        dcurve = [curve.derivative(o=o) for o in range(1, order + 1)]

        s = curve(tau)
        ds = [dcurve_k(tau) for dcurve_k in dcurve]
        flat_output = cd.horzcat(s, *ds)

        return flat_output

    def compute_control_points(self, X):
        """compute the control points of flat trajectory X with shape(nr control points, flat output)"""
        assert X.shape == (self.K, 2)

        A = self.evaluate_base(np.linspace(0, 1, num=self.K))  # shape=(K,K)
        return cd.inv(A) @ X

    def compute_mixed_control_points(self, X, X_dot0, X_dot1, kk, index=0):
        """
        Compute the control points with constraints to the derivative at the start and end point to result in better fitting
        Does not work atm.
        """
        A = self.evaluate_base(np.linspace(0, 1, num=self.K))  # shape=(K,K)
        # X_dot = X[1] - X[0]
        # X_dot = X_dot.copy()
        if index == 0:
            A[1, :] = np.array([-1, 1] + [0] * 82) * self.degree / kk
            X[1, :] = X_dot1

            A[0, :] = np.array([0] * 82 + [-1, 1]) * self.degree / kk
            X[0, :] = X_dot0

        return cd.inv(A) @ X

    def evaluate_base(self, tau):
        return cd.densify(self.base(tau))
