# %%
import matplotlib.pyplot as plt
import numpy as np
from environments.kinematic_car import KinematicCar

# %%
car = KinematicCar(dt=0.01)
car.state = [0, 0, 0, 0, 0]  # [x, y, ψ, vx, vy, ω, δ]

state = car.state
states = [state]

for i in range(500):
    if i < 50:
        u = [10, 0.0]  # [v, psi]
    elif i < 100:
        u = [0, 1]
    else:
        u = [0.1, 0.0]

    state = car.step(u)
    states.append(state)

# %%
fig, axs = plt.subplots(3, 1)
states_array = np.array(states)
axs[0].plot(states_array[:, 0:2], label=["x", "y"])
axs[1].plot(states_array[:, 2] * 180 / np.pi, label="psi")
axs[2].plot(states_array[:, 4], label="v")
[ax.legend() for ax in axs]
fig.tight_layout()
# %%
fig, ax = plt.subplots()
ax.plot(states_array[:, 0], states_array[:, 1])
# ax.set_aspect("equal")

# %%
print(states_array[:100, :])
# %%
