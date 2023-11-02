# %%
import matplotlib.pyplot as plt
import numpy as np
from environments.kinematic_car import KinematicCar

# %%
car = KinematicCar(dt=0.01)
car.state = [0, 0, np.pi / 2]  # [x, y, ψ, vx, vy, ω, δ]

state = car.state
states = [state]

for i in range(200):
    if i < 50:
        u = [0.1, -1]  # [Fx_m, dδ]
    elif i < 100:
        u = [0.1, 1]
    else:
        u = [0.1, 0.0]

    state = car.step(u)
    states.append(state)

# %%
fig, axs = plt.subplots(2, 1)
states_array = np.array(states)
axs[0].plot(states_array[:, 0:2], label=["x", "y"])
axs[1].plot(states_array[:, 2] * 180 / np.pi, label="psi")
[ax.legend() for ax in axs]
fig.tight_layout()
# %%
fig, ax = plt.subplots()
ax.plot(states_array[:, 0], states_array[:, 1])
# ax.set_aspect("equal")

# %%
