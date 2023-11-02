# %% imports
import casadi as cd
import matplotlib.pyplot as plt
import numpy as np
from environments.kinematic_car import KinematicCar
from optimal_control.MPC import MPC
from optimal_control.ocp import Ocp

# %% build optimal control problem (OCP)
car = KinematicCar(dt=0.05)  # s=[x, y, ψ], u=[Fx_m, δ]

N = 50
ocp = Ocp(
    car.nx,
    car.nu,
    N=N,
    F=car.F,
    T=car.dt * N,
    terminal_constraint=True,
    integrator="euler",
)

Q = np.diag([1e-2, 1e-2, 0])
R = np.diag([1e-1, 1e-1])

# P = np.diag([10, 10, 1])
P = np.diag([0, 0, 0])

ocp.running_cost = (ocp.x - ocp.x_ref).T @ Q @ (ocp.x - ocp.x_ref) + ocp.u.T @ R @ ocp.u
ocp.terminal_cost = (ocp.x - ocp.x_ref).T @ P @ (ocp.x - ocp.x_ref)

# constraints
max_steering_angle = 1.0
max_steering_rate = np.pi / 4 / (5 * car.dt)
max_v = 0.5
ocp.subject_to(ocp.bounded(0, ocp.U[0, :], max_v))
ocp.subject_to(ocp.bounded(-max_steering_angle, ocp.U[1, :], max_steering_angle))
# ocp.subject_to(ocp.bounded(-max_steering_angle, ocp.X[6, :], max_steering_angle))
# ocp.subject_to(ocp.bounded(-max_steering_rate, ocp.U[1, :], max_steering_rate))

add_obstacle = True
if add_obstacle:
    obstacle_centre = [0.2, 0.2]
    obstacle_radius = 0.2
    ocp.subject_to(
        cd.sum1((ocp.X[0:2, :] - obstacle_centre) ** 2) > obstacle_radius**2
    )


# %% solve
init_state = [0, 0, np.pi / 2]
goal_state = [0.5, 0.5, 0]
U, X, info = ocp.solve(init_state, goal_state, lin_interpol=True)

# %% show result
fig, axs = plt.subplots(2, 1)
states_array = X.T
axs[0].plot(states_array[:, 0:2], label=["x", "y"])
axs[1].plot(states_array[:, 2] * 180 / np.pi, label="psi")
[ax.legend() for ax in axs]
fig.tight_layout()

fig, ax = plt.subplots()
ax.plot(states_array[:, 0], states_array[:, 1])
if add_obstacle:
    circle = plt.Circle(obstacle_centre, obstacle_radius, color="k", fill=False)
    ax.add_patch(circle)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")

# %% MPC
mpc = MPC(ocp)

mpc.X_init_guess = X
current_state = init_state

for _ in range(ocp.N):
    u, info = mpc(current_state, goal_state)
    current_state = info["X_sol"][:, 1]

X_closed_loop = np.array(mpc.X_trajectory)
U_closed_loop = np.array(mpc.U_trajectory)
# %%
fig, ax = plt.subplots()
ax.plot(X_closed_loop[:, 0], X_closed_loop[:, 1])
if add_obstacle:
    circle = plt.Circle(obstacle_centre, obstacle_radius, color="k", fill=False)
    ax.add_patch(circle)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")

# %%
