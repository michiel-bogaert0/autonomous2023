import gif
import matplotlib.pyplot as plt
import numpy as np
from environments.Env import Env
from environments.env_utils import Space
from tqdm import tqdm

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

        high_state_space = np.array([np.inf, np.inf, np.inf])
        self.state_space = Space(low=-high_state_space, high=high_state_space)

        high_obs_space = np.array([np.inf, np.inf, np.inf])
        self.observation_space = Space(low=-high_obs_space, high=high_obs_space)

        self.F = self.discretize(self.dynamics, self.dt)

    def dynamics(self, s, u):
        """
        s=[x, y, ψ]
        u=[v, δ]
        """
        # x, y, psi, vx, vy, omega, delta = s[0], s[1], s[2], s[3], s[4], s[5], s[6]
        _, _, psi = s[0], s[1], s[2]
        v = u[0]
        delta = u[1]

        beta = np.arctan(self.l_r * np.tan(delta) / self.L)

        dx = v * np.cos(beta + psi)
        dy = v * np.sin(beta + psi)
        dpsi = v / self.L * np.cos(beta) * np.tan(delta)

        return np.array([dx, dy, dpsi])

    def reset(self):
        pass

    def obs(self, x):
        pass


# TODO use modulo to keep pendulum in the frame
def animate(states, name="furuta", Lp=0.129, r=0.085, T=None):
    @gif.frame
    def plot(i):
        fig, ax = plt.subplots()
        ax.set_aspect("equal")

        theta = states[i][0]
        alpha = states[i][1]
        ax.scatter(theta * r, 0, c="red")
        ax.plot([theta * r, theta * r + np.sin(alpha) * Lp], [0, -np.cos(alpha) * Lp])

        ax.set_ylim(-1.2 * Lp, 1.2 * Lp)
        ax.set_xlim(-np.pi / 2 * r, np.pi / 2 * r)

        x_tick = np.linspace(-1 / 2, 1 / 2, num=5)
        x_label = [f"${r} / pi$" for r in x_tick]
        ax.set_xticks(x_tick * np.pi * r)
        ax.set_xticklabels(x_label, fontsize=10)

    frames = []
    for i in tqdm(range(len(states))):
        frame = plot(i)
        frames.append(frame)

    if T is None:
        duration = 30
    else:
        duration = (T / (len(states) - 1)) * 1000

    gif.save(frames, name + ".gif", duration=duration)
