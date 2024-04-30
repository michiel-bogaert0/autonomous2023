import numpy as np
from environments_gen.Env import Env
from environments_gen.env_utils import Space


class BicycleModelSpline(Env):
    def __init__(self, dt=0.01, R=0.1, L=0.72, Lr=0.5, mu=0.1, DC=0.025):
        self.dt = dt
        self.R = R
        self.L = L
        self.Lr = Lr
        self.mu = mu
        self.DC = DC

        self.action_space = Space(
            low=[-np.inf, -np.inf, np.inf], high=[np.inf, np.inf, np.inf]
        )

        high_state_space = np.array([np.inf, np.inf, 2 * np.pi, np.pi, 0, 0])
        high_state_space = np.array([np.inf, np.inf, 2 * np.pi, np.pi, np.inf, 1])
        self.state_space = Space(low=-high_state_space, high=high_state_space)

        high_state_space = np.array([np.inf, np.inf, 2 * np.pi, np.pi, 0, 0])
        high_obs_space = np.array([np.inf, np.inf, 2 * np.pi, np.pi, np.inf, 1])
        # high_obs_space = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
        self.observation_space = Space(low=-high_obs_space, high=high_obs_space)

        self.F = self.discretize(self.dynamics, self.dt, integrator="rk4", M=4)

    def dynamics(self, s, u):
        # Inputs
        # x = s[0]
        # y = s[1]
        theta = s[2]
        zeta = s[3]
        v = s[4]
        # tau = s[5]
        phi = u[1]
        alpha = u[0]
        vk = u[2]

        a = alpha * self.R

        # Outputs
        omega = v * np.tan(zeta) / self.L

        dx = v * np.cos(theta)
        dy = v * np.sin(theta)

        dv = a

        dtau = vk / self.dt  # cancel out dt

        return np.array([dx, dy, omega, phi, dv, dtau])

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
