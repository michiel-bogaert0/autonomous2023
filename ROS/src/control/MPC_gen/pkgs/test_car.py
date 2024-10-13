# %%
import matplotlib.pyplot as plt
import numpy as np
from environments_gen.bicycle_model import BicycleModel
from environments_gen.bicycle_model_spline import BicycleModelSpline
from environments_gen.kinematic_car import KinematicCar

###########################################################
# Useful file for model verification, however some things #
# in here are thesis-leftovers                            #
###########################################################

# %%

X0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/X0_02.npy"
)
U0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/U0_02.npy"
)
X0_02 = X0
print(X0)
# %%
car = KinematicCar(dt=0.05)
car = BicycleModel(dt=0.05)
car = BicycleModelSpline(dt=0.02)
car.state = X0[:, 0]
# car.state[2] = - np.pi
# car.state = [0, 0, 0, 0, 0, 0]  # [x, y, ψ, vx, vy, ω, δ]

state = car.state
states = [state]

# %%

for i in range(len(U0[0])):
    u = U0[:, i]
    print(u)
    # car.state = X0[:, i]
    print(car.state)
    state = car.step(u)
    print(state)
    print(X0[:, i + 1])
    states.append(state)

# %%

# for i in range(50):
#     if i < 50:
#         u = [200, 0.5, 0.02]  # [v, psi]
#         print(f"v: {state[4]}")
#         print(f"zeta: {state[3]}")
#         print(f"tan(zeta): {np.tan(state[3])}")
#     elif i < 100:
#         u = [-5, np.pi]
#     else:
#         u = [0.1, 0.0]

#     state = car.step(u)
#     states.append(state)

# %%
fig, axs = plt.subplots(8, 1)
states_array = np.array(states)
print(states_array[:, 4])
print(X0[3, :])
axs[0].plot(states_array[:, 0], states_array[:, 1], label=["x", "y"])
axs[1].plot(states_array[:, 2], label="heading")
axs[2].plot(X0[2, :], label="heading GT")
axs[3].plot(states_array[:, 3], label="steering angle")
axs[4].plot(X0[3, :], label="steering_angle GT")
axs[5].plot(states_array[:, 4], label="v")
axs[6].plot(X0[4, :], label="v GT")
axs[7].plot(states_array[:, 5], label="tau")
states_array_02 = states_array
# [ax.legend() for ax in axs]
# fig.tight_layout()
# %%
fig, ax = plt.subplots()
ax.plot(states_array[:, 0], states_array[:, 1], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")

# %%
fig, ax = plt.subplots()
ax.plot(X0[0, :], X0[1, :], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")

# %%
print(X0[:100, :])
print(states_array[:100, :])


# %%

X0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/X0_05.npy"
)
U0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/U0_05.npy"
)
X0_mpcc = X0

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

# %%

for i in range(len(U0[0])):
    u = U0[:, i]
    print(u)
    # car.state = X0[:, i]
    print(car.state)
    state = car.step(u)
    print(state)
    print(X0[:, i + 1])
    states.append(state)

# %%

# for i in range(50):
#     if i < 50:
#         u = [200, 0.5, 0.02]  # [v, psi]
#         print(f"v: {state[4]}")
#         print(f"zeta: {state[3]}")
#         print(f"tan(zeta): {np.tan(state[3])}")
#     elif i < 100:
#         u = [-5, np.pi]
#     else:
#         u = [0.1, 0.0]

#     state = car.step(u)
#     states.append(state)

# %%
fig, axs = plt.subplots(8, 1)
states_array = np.array(states)
print(states_array[:, 4])
print(X0[3, :])
axs[0].plot(states_array[:, 0], states_array[:, 1], label=["x", "y"])
axs[1].plot(states_array[:, 2], label="heading")
axs[2].plot(X0[2, :], label="heading GT")
axs[3].plot(states_array[:, 3], label="steering angle")
axs[4].plot(X0[3, :], label="steering_angle GT")
axs[5].plot(states_array[:, 4], label="v")
axs[6].plot(X0[4, :], label="v GT")
axs[7].plot(states_array[:, 5], label="tau")
states_array_05 = states_array
# [ax.legend() for ax in axs]
# fig.tight_layout()
# %%
fig, ax = plt.subplots()
ax.plot(states_array[:, 0], states_array[:, 1], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")

# %%
fig, ax = plt.subplots()
ax.plot(X0[0, :], X0[1, :], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")

# %%
fig, ax = plt.subplots()
plt.rcParams["text.usetex"] = False

# ax = plt.gca()
ax.axis("off")
# ax.set_aspect("equal", "datalim")
ax.plot(X0[0, :], X0[1, :], label="GT", c="r")
ax.plot(states_array[:, 0], states_array[:, 1], label="dt = 0.02, N=425")
ax.plot(states_array_05[:, 0], states_array_05[:, 1], label="dt = 0.05, N=170")
ax.set_aspect("equal")

# For left corner
# plt.xlim(-50, -30)
# plt.ylim(-17, 0)

# For the right corner
# plt.xlim(30, 50)
# plt.ylim(-19, -2)

# ax.legend()

# increase plot size
fig.set_size_inches(12, 12)
# %%
X0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/X0_05_ref.npy"
)
U0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/U0_05_ref.npy"
)

X0_ref = X0
print(X0)
# %%
car = KinematicCar(dt=0.05)
car = BicycleModel(dt=0.05)
# car = BicycleModelSpline(dt=0.02)
car.state = X0[:, 0]
# car.state[2] = - np.pi
# car.state = [0, 0, 0, 0, 0, 0]  # [x, y, ψ, vx, vy, ω, δ]

state = car.state
states = [state]

# %%

for i in range(len(U0[0])):
    u = U0[:, i]
    print(u)
    # car.state = X0[:, i]
    print(car.state)
    state = car.step(u)
    print(state)
    print(X0[:, i + 1])
    states.append(state)

# %%

# for i in range(50):
#     if i < 50:
#         u = [200, 0.5, 0.02]  # [v, psi]
#         print(f"v: {state[4]}")
#         print(f"zeta: {state[3]}")
#         print(f"tan(zeta): {np.tan(state[3])}")
#     elif i < 100:
#         u = [-5, np.pi]
#     else:
#         u = [0.1, 0.0]

#     state = car.step(u)
#     states.append(state)

# %%
fig, axs = plt.subplots(8, 1)
states_array = np.array(states)
print(states_array[:, 4])
print(X0[3, :])
axs[0].plot(states_array[:, 0], states_array[:, 1], label=["x", "y"])
axs[1].plot(states_array[:, 2], label="heading")
axs[2].plot(X0[2, :], label="heading GT")
axs[3].plot(states_array[:, 3], label="steering angle")
axs[4].plot(X0[3, :], label="steering_angle GT")
axs[5].plot(states_array[:, 4], label="v")
axs[6].plot(X0[4, :], label="v GT")
# axs[7].plot(states_array[:, 5], label="tau")
# [ax.legend() for ax in axs]
# fig.tight_layout()
states_array_05_ref = states_array
# %%
fig, ax = plt.subplots()
ax.plot(states_array[:, 0], states_array[:, 1], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")

# %%
fig, ax = plt.subplots()
ax.plot(X0[0, :], X0[1, :], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")
# %%

X0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/X0.npy"
)
U0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/U0.npy"
)
X0_02_ref = X0
print(X0)
# %%
car = KinematicCar(dt=0.05)
car = BicycleModel(dt=0.02)
# car = BicycleModelSpline(dt=0.02)
car.state = X0[:, 0]
# car.state[2] = - np.pi
# car.state = [0, 0, 0, 0, 0, 0]  # [x, y, ψ, vx, vy, ω, δ]

state = car.state
states = [state]

# %%

for i in range(len(U0[0])):
    u = U0[:, i]
    print(u)
    # car.state = X0[:, i]
    print(car.state)
    state = car.step(u)
    print(state)
    print(X0[:, i + 1])
    states.append(state)

# %%

# for i in range(50):
#     if i < 50:
#         u = [200, 0.5, 0.02]  # [v, psi]
#         print(f"v: {state[4]}")
#         print(f"zeta: {state[3]}")
#         print(f"tan(zeta): {np.tan(state[3])}")
#     elif i < 100:
#         u = [-5, np.pi]
#     else:
#         u = [0.1, 0.0]

#     state = car.step(u)
#     states.append(state)

# %%
fig, axs = plt.subplots(8, 1)
states_array = np.array(states)
print(states_array[:, 4])
print(X0[3, :])
axs[0].plot(states_array[:, 0], states_array[:, 1], label=["x", "y"])
axs[1].plot(states_array[:, 2], label="heading")
axs[2].plot(X0[2, :], label="heading GT")
axs[3].plot(states_array[:, 3], label="steering angle")
axs[4].plot(X0[3, :], label="steering_angle GT")
axs[5].plot(states_array[:, 4], label="v")
axs[6].plot(X0[4, :], label="v GT")
# axs[7].plot(states_array[:, 5], label="tau")
# [ax.legend() for ax in axs]
# fig.tight_layout()
states_array_02_ref = states_array
# %%
fig, ax = plt.subplots()
ax.plot(states_array[:, 0], states_array[:, 1], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")

# %%
fig, ax = plt.subplots()
ax.plot(X0[0, :], X0[1, :], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")
# %%
# %%
fig, ax = plt.subplots()
plt.rcParams["text.usetex"] = False

# ax = plt.gca()
ax.axis("off")
# ax.set_aspect("equal", "datalim")
ax.plot(X0[0, :], X0[1, :], label="GT", c="black")
# ax.plot(states_array_02[:, 0], states_array_02[:, 1], label="MPCC: dt = 0.02, N=1075")
ax.plot(states_array_05[:, 0], states_array_05[:, 1], label="MPCC")
# ax.plot(states_array_02_ref[:, 0], states_array_02_ref[:, 1], label="Reference: dt = 0.02, N=1075")
ax.plot(
    states_array_05_ref[:, 0], states_array_05_ref[:, 1], label="Reference-Tracking"
)
ax.set_aspect("equal")

# For left corner
# plt.xlim(-50, -30)
# plt.ylim(-21, 0)

# For the right corner
# plt.xlim(30, 50)
# plt.ylim(-21, 0)

ax.legend(bbox_to_anchor=(1.05, -0.2), loc="lower right", fontsize=12)

# increase plot size
fig.set_size_inches(10, 10)
# %%
# plt.plot(states_array_05[:, 2], label="MPCC")
plt.plot(X0_mpcc[3, :], label="steering_angle GT")
plt.plot(X0_ref[3, :], label="steering_angle GT ref")
plt.plot(X0_02[2, :], label="steering_angle GT 02")
plt.plot(X0_02_ref[2, :], label="steering_angle GT ref 02")
# plt.plot(states_array_05_ref[:, 2], label="Reference")
plt.xlim(0, 100)
plt.ylim(-0.2, 0.2)
plt.legend()

# %%
# Calculate eucl distance between GT and MPCC
dist = np.sqrt(
    (X0_mpcc[0, :] - states_array_05[:, 0]) ** 2
    + (X0_mpcc[1, :] - states_array_05[:, 1]) ** 2
)
print(np.mean(dist))
print(np.max(dist))

dist = np.sqrt(
    (X0_ref[0, :] - states_array_05_ref[:, 0]) ** 2
    + (X0_ref[1, :] - states_array_05_ref[:, 1]) ** 2
)
print(np.mean(dist))
print(np.max(dist))
# %%
X0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/X0.npy"
)
U0 = np.load(
    "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC_gen/data_gen/U0.npy"
)
X0_02_ref = X0
print(X0)
# %%
car = KinematicCar(dt=0.05)
car = BicycleModel(dt=0.02)
car = BicycleModelSpline(dt=0.05)
car.state = X0[:, 0]
# car.state[2] = - np.pi
# car.state = [0, 0, 0, 0, 0, 0]  # [x, y, ψ, vx, vy, ω, δ]

state = car.state
states = [state]

# %%

for i in range(len(U0[0])):
    u = U0[:, i]
    print(u)
    # car.state = X0[:, i]
    print(car.state)
    state = car.step(u)
    print(state)
    print(X0[:, i + 1])
    states.append(state)

# %%

# for i in range(50):
#     if i < 50:
#         u = [200, 0.5, 0.02]  # [v, psi]
#         print(f"v: {state[4]}")
#         print(f"zeta: {state[3]}")
#         print(f"tan(zeta): {np.tan(state[3])}")
#     elif i < 100:
#         u = [-5, np.pi]
#     else:
#         u = [0.1, 0.0]

#     state = car.step(u)
#     states.append(state)

# %%
fig, axs = plt.subplots(8, 1)
states_array = np.array(states)
print(states_array[:, 4])
print(X0[3, :])
axs[0].plot(states_array[:, 0], states_array[:, 1], label=["x", "y"])
axs[1].plot(states_array[:, 2], label="heading")
axs[2].plot(X0[2, :], label="heading GT")
axs[3].plot(states_array[:, 3], label="steering angle")
axs[4].plot(X0[3, :], label="steering_angle GT")
axs[5].plot(states_array[:, 4], label="v")
axs[6].plot(X0[4, :], label="v GT")
axs[7].plot(states_array[:, 5], label="tau")
plt.xlim(150, 250)
# [ax.legend() for ax in axs]
# fig.tight_layout()
states_array_02_ref = states_array
# %%
fig, ax = plt.subplots()
ax.plot(states_array[:, 0], states_array[:, 1], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")

# %%
fig, ax = plt.subplots()
ax.plot(X0[0, :], X0[1, :], "o")
# plt.xlim(-40, 12)
ax.set_aspect("equal")
# %%
# %%
fig, ax = plt.subplots()
plt.rcParams["text.usetex"] = False

# ax = plt.gca()
ax.axis("off")
# ax.set_aspect("equal", "datalim")
ax.plot(X0[0, :], X0[1, :], label="GT", c="black")
# ax.plot(states_array_02[:, 0], states_array_02[:, 1], label="MPCC: dt = 0.02, N=1075")
ax.plot(states_array_05[:, 0], states_array_05[:, 1], label="MPCC")
# ax.plot(states_array_02_ref[:, 0], states_array_02_ref[:, 1], label="Reference: dt = 0.02, N=1075")
ax.plot(
    states_array_05_ref[:, 0], states_array_05_ref[:, 1], label="Reference-Tracking"
)
ax.set_aspect("equal")

# For left corner
# plt.xlim(-50, -30)
# plt.ylim(-21, 0)

# For the right corner
# plt.xlim(30, 50)
# plt.ylim(-21, 0)

ax.legend(bbox_to_anchor=(1.05, -0.2), loc="lower right", fontsize=12)

# increase plot size
fig.set_size_inches(10, 10)
# %%
# plt.plot(states_array_05[:, 2], label="MPCC")
plt.plot(X0_mpcc[3, :], label="steering_angle GT")
plt.plot(X0_ref[3, :], label="steering_angle GT ref")
plt.plot(X0_02[2, :], label="steering_angle GT 02")
plt.plot(X0_02_ref[2, :], label="steering_angle GT ref 02")
# plt.plot(states_array_05_ref[:, 2], label="Reference")
plt.xlim(0, 100)
plt.ylim(-0.2, 0.2)
plt.legend()

# %%
# Calculate eucl distance between GT and MPCC
dist = np.sqrt(
    (X0_mpcc[0, :] - states_array_05[:, 0]) ** 2
    + (X0_mpcc[1, :] - states_array_05[:, 1]) ** 2
)
print(np.mean(dist))
print(np.max(dist))

dist = np.sqrt(
    (X0_ref[0, :] - states_array_05_ref[:, 0]) ** 2
    + (X0_ref[1, :] - states_array_05_ref[:, 1]) ** 2
)
print(np.mean(dist))
print(np.max(dist))
