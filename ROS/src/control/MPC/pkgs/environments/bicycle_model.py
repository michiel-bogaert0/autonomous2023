import numpy as np
from environments.Env import Env
from environments.env_utils import Space

############################################
#       THIS DOES NOT WORK YET            #
############################################


class BicycleModel(Env):
    def __init__(self, dt=0.01, R=0.1, L=1.0, Lr=0.5, mu=0.1, DC=0.025):
        self.dt = dt
        self.R = R
        self.L = L
        self.Lr = Lr
        self.mu = mu
        self.DC = DC

        self.a = 0
        self.v = 0

        self.x = 0
        self.y = 0
        self.theta = 0
        self.zeta = 0
        self.ang_vel = 0
        self.phi = 0

        self.action_space = Space(low=[-100, -100], high=[100, 100])

        high_state_space = np.array([np.inf, np.inf, np.inf, np.inf])
        self.state_space = Space(low=-high_state_space, high=high_state_space)

        high_obs_space = np.array([np.inf, np.inf, np.inf, np.inf])
        self.observation_space = Space(low=-high_obs_space, high=high_obs_space)

        self.F = self.discretize(self.dynamics, self.dt)

    def dynamics(self, s, u):
        # Inputs
        phi = u[0]
        alpha = u[1]

        # x, y, psi, v = s[0], s[1], s[2], s[3]
        v = s[3]

        # Angular velocity (of wheel) and steering angle
        self.zeta += phi * self.dt

        # drag_acc = self.DC * np.power(v, 2)
        # friction_acc = casadi.if_else(casadi.power(v, 2) > 0.001 ** 2, self.mu, 0)

        a = alpha * self.R  # - drag_acc - friction_acc

        v += a * self.dt  # v of CoG (+ drag + friction)
        self.ang_vel = v / self.R

        # Outputs
        dpsi = self.v * np.tan(self.zeta) / self.L
        self.theta += dpsi * self.dt
        dx = self.v * np.cos(self.theta)
        dy = self.v * np.sin(self.theta)

        dv = self.a * self.dt

        return np.array([dx, dy, dpsi, dv])

    def reset(self):
        self.a = 0
        self.v = 0

        self.x = 0
        self.y = 0
        self.theta = 0
        self.zeta = 0
        self.ang_vel = 0
        self.phi = 0
        self.omega = 0

    def obs(self, x):
        return np.sqrt((x[0] - x[1]) ** 2 + (x[2] - x[3]) ** 2)
