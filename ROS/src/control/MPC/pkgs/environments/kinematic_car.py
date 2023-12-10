import numpy as np
from environments.Env import Env
from environments.env_utils import Space

g = 9.81


class KinematicCar(Env):
    def __init__(
        self,
        dt=0.01,
        l_r=0.033,
        L=0.12,
    ) -> None:
        self.l_r = l_r
        self.L = L

        self.dt = dt

        self.action_space = Space(low=[0, -np.pi / 4], high=[2, np.pi / 4])

        high_state_space = np.array([np.inf, np.inf, np.inf, np.inf])
        self.state_space = Space(low=-high_state_space, high=high_state_space)

        high_obs_space = np.array([np.inf, np.inf, np.inf, np.inf])
        self.observation_space = Space(low=-high_obs_space, high=high_obs_space)

        self.F = self.discretize(self.dynamics, self.dt)

    def dynamics(self, s, u):
        """
        s=[x, y, ψ]
        u=[v, δ]
        """
        psi = s[2]
        v = u[0]
        delta = u[1]

        beta = np.arctan(self.l_r * np.tan(delta) / self.L)

        dx = v * np.cos(beta + psi)
        dy = v * np.sin(beta + psi)
        dpsi = v / self.L * np.cos(beta) * np.tan(delta)

        return np.array([dx, dy, dpsi, v])

    def reset(self):
        pass

    def obs(self, x):
        return np.sqrt((x[0] - x[1]) ** 2 + (x[2] - x[3]) ** 2)
