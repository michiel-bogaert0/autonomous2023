# %%
import matplotlib.pyplot as plt
import numpy as np
from environments_gen.bicycle_model import BicycleModel
from environments_gen.bicycle_model_spline import BicycleModelSpline
from environments_gen.kinematic_car import KinematicCar

# %%

X0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/X0.npy"
)
U0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/U0.npy"
)

print(X0)
# %%
car = KinematicCar(dt=0.05)
car = BicycleModel(dt=0.05)
car = BicycleModelSpline(dt=0.05)
car.state = X0[:, 0]
# car.state[2] = - np.pi
# car.state = [0, 0, 0, 0, 0, 0]  # [x, y, ψ, vx, vy, ω, δ]

state = car.state
states = [state]

for i in range(len(U0[0])):
    u = U0[:, i]
    print(u)
    # car.state = X0[:, i]
    print(car.state)
    state = car.step(u)
    print(state)
    print(X0[:, i + 1])
    states.append(state)

# for i in range(50):
#     if i < 50:
#         u = [0.2, 0.0, 0.02]  # [v, psi]
#     elif i < 100:
#         u = [-5, np.pi]
#     else:
#         u = [0.1, 0.0]

#     state = car.step(u)
#     states.append(state)

# %%
fig, axs = plt.subplots(5, 1)
states_array = np.array(states)
print(states_array[:, 2])
axs[0].plot(states_array[:, 0], states_array[:, 1], label=["x", "y"])
axs[1].plot(states_array[:, 2], label="heading")
axs[2].plot(states_array[:, 3], label="steering angle")
axs[3].plot(states_array[:, 4], label="v")
axs[4].plot(states_array[:, 5], label="tau")
[ax.legend() for ax in axs]
# fig.tight_layout()
# %%
fig, ax = plt.subplots()
ax.plot(states_array[:, 0], states_array[:, 1], "o")
ax.set_aspect("equal")

# %%
fig, ax = plt.subplots()
ax.plot(X0[0, :], X0[1, :], "o")
ax.set_aspect("equal")

# %%
print(states_array[:100, :])
# %%
