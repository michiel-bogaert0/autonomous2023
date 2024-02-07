import numpy as np
from environments.Env import Env
from environments.env_utils import Space

g = 9.81


class KinematicCar(Env):
    def __init__(
        self,
        dt=0.01,
        l_r=0.5,
        L=1.0,
    ) -> None:
        self.l_r = l_r
        self.L = L

        self.dt = dt

        self.action_space = Space(low=[0, -np.pi], high=[20, np.pi])

        high_state_space = np.array([np.inf, np.inf, np.inf, np.inf, np.inf])
        self.state_space = Space(low=-high_state_space, high=high_state_space)

        high_obs_space = np.array([np.inf, np.inf, np.inf, np.inf, np.inf])
        self.observation_space = Space(low=-high_obs_space, high=high_obs_space)

        self.F = self.discretize(self.dynamics, self.dt)

    def dynamics(self, s, u):
        """
        s=[x, y, theta, delta, v]
        u=[a, psi]
        """
        # x = s[0]
        # y = s[1]
        theta = s[2]
        delta = s[3]
        v = s[4]
        a = u[0]
        psi = u[1]

        beta = np.arctan(self.l_r * np.tan(delta) / self.L)

        dx = v * np.cos(beta + theta)
        dy = v * np.sin(beta + theta)
        dtheta = v / self.L * np.tan(delta) * np.cos(beta)
        ddelta = psi
        dv = a

        return np.array([dx, dy, dtheta, ddelta, dv])

    def reset(self):
        pass

    def obs(self, x):
        return np.sqrt((x[0] - x[1]) ** 2 + (x[2] - x[3]) ** 2)
