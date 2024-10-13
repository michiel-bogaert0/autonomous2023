# import bagpy
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# import rosbag
import yaml
from bagpy import bagreader

# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Path
from scipy.interpolate import interp1d

#######################################################
# This was a file for my thesis,                      #
# can be useful for plotting so leaving it here       #
#######################################################


def read_yaml(path):
    path = path

    with open(path, "r") as file:
        arr = yaml.safe_load(file)
    return arr


def interpolate_arr(arr, n=3):
    distance = np.cumsum(np.sqrt(np.sum(np.diff(arr, axis=0) ** 2, axis=1)))
    distance = np.insert(distance, 0, 0) / distance[-1]

    alpha = np.linspace(0, 1, len(arr) * n)
    interpolator = interp1d(distance, arr, kind="cubic", axis=0)
    arr = interpolator(alpha)

    return arr


def calculate_time(path, dt, vel):
    time = 0
    for i in range(1, len(path)):
        time += (
            np.sqrt(
                (path[i][0] - path[i - 1][0]) ** 2 + (path[i][1] - path[i - 1][1]) ** 2
            )
            / vel
        )
    # time += dt
    return time


# plt.rcParams['text.usetex'] = True

ax = plt.gca()
ax.axis("off")
ax.set_aspect("equal", "datalim")

# Get left and right boundaries from yaml
GT_left_boundary = []
GT_right_boundary = []

path_GT_map = "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC/data/map_fsi.yaml"
path_GT_centerline = "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC/data/centerline_fsi.yaml"

GT_centerline = []

arr = read_yaml(path_GT_centerline)
for pose in arr["poses"]:
    GT_centerline.append([pose["pose"]["position"]["x"], pose["pose"]["position"]["y"]])

GT_centerline = np.array(GT_centerline)

arr = read_yaml(path_GT_map)
for obs in arr["observations"]:
    if obs["observation"]["observation_class"] == 0:
        GT_left_boundary.append(
            [
                obs["observation"]["location"]["x"],
                obs["observation"]["location"]["y"],
            ]
        )
    elif obs["observation"]["observation_class"] == 1:
        GT_right_boundary.append(
            [
                obs["observation"]["location"]["x"],
                obs["observation"]["location"]["y"],
            ]
        )

# Close boundaries
GT_left_boundary.append(GT_left_boundary[0])
GT_right_boundary.append(GT_right_boundary[0])

GT_left_boundary = interpolate_arr(GT_left_boundary)
GT_right_boundary = interpolate_arr(GT_right_boundary)

# plt.plot(GT_centerline.T[0], GT_centerline.T[1], c="black", label="Centerline")
# plt.plot(GT_centerline.T[0][0], GT_centerline.T[1][0], 'o', c="black")

plt.plot(GT_left_boundary.T[0], GT_left_boundary.T[1], c="b")
plt.plot(GT_right_boundary.T[0], GT_right_boundary.T[1], c="y")

path_generation = "/home/zander/Documents/ugent_racing/autonomous2023/ROS/src/control/MPC/data/solution_gen_ref_halfspaces_fsc.npz"
data = np.load(path_generation)
X_sol_intermediate = data["X_sol_intermediate"]
last_sol = X_sol_intermediate[-1]
# plt.plot(last_sol[0], last_sol[1], "--", c="red", label="Raceline")
# plt.plot(last_sol[0], last_sol[1], c="black", label="Raceline")
# plt.plot(last_sol[0][0], last_sol[1][0], 'o', c="black")

# plt.show()

b = bagreader(
    "/home/zander/Documents/ugent_racing/rosbags/reference_tracking_halfspaces.bag"
)

# get the list of topics
print(b.topic_table)

ms = b.message_by_topic("/ugr/car/odometry/filtered/map")
odom = pd.read_csv(ms)
print(odom.head())
timestamp = 50698.504  # ref circles
timestamp = 57745.658  # ref halfspaces
timestamp_max = 57772.252  # ref halfspaces
# timestamp = 31843.425 # ref racing halfspaces
# timestamp_max = 31868.637 # ref racing halfspaces
# timestamp =  273364.914 # ref center halfspaces fsg
# timestamp_max = 273402.137 # ref center halfspaces fsg
# timestamp = 274128.082 # ref raceline halfspaces fsg
# timestamp_max = 274163.358 # ref raceline halfspaces fsg
# timestamp = 273528.208 # ref center halfspaces fsc
# timestamp_max = 273561.852 # ref center halfspaces fsc
# timestamp = 273974.23 # ref raceline halfspaces fsc
# timestamp_max = 274006.211 # ref raceline halfspaces fsc
odom = odom[odom["Time"] > timestamp]
odom = odom[odom["Time"] < timestamp_max]
plt.plot(
    odom["pose.pose.position.x"],
    odom["pose.pose.position.y"],
    c="green",
    label="Driven path with halfspaces",
)

odom["relative_time"] = odom["Time"] - odom["Time"].iloc[0]
odom_halfspaces = odom

ms_steering = b.message_by_topic("/ugr/car/steering_velocity_controller/command")
steering = pd.read_csv(ms_steering)
print(steering.head())
steering = steering[steering["Time"] > timestamp]
steering = steering[steering["Time"] < timestamp_max]

steering["relative_time"] = steering["Time"] - steering["Time"].iloc[0]
steering_halfspaces = steering

b = bagreader(
    "/home/zander/Documents/ugent_racing/rosbags/reference_tracking_circles.bag"
)

# get the list of topics
print(b.topic_table)

ms = b.message_by_topic("/ugr/car/odometry/filtered/map")
odom = pd.read_csv(ms)
print(odom.head())
timestamp = 50698.204  # ref circles
# timestamp = 57745.658 # ref halfspaces
timestamp_max = 57772.252  # ref halfspaces
# # timestamp = 31843.425 # ref racing halfspaces
# # timestamp_max = 31868.637 # ref racing halfspaces
# timestamp = 201512.021 # ref racing circles
# timestamp_max = 201537.274 # ref racing circles
# timestamp = 273250.997 # ref center circles fsg
# timestamp_max = 273288.09 # ref center circles fsg
# timestamp = 274247.232 # ref raceline circles fsg
# timestamp_max = 274282.539 # ref raceline circles fsg
# timestamp = 273618.515 # ref center circles fsc
# timestamp_max = 273651.931 # ref center circles fsc
# timestamp = 273857.959 # ref raceline circles fsc
# timestamp_max = 273889.853 # ref raceline circles fsc
odom = odom[odom["Time"] > timestamp]
odom = odom[odom["Time"] < timestamp_max]
plt.plot(
    odom["pose.pose.position.x"],
    odom["pose.pose.position.y"],
    c="m",
    label="Driven path with circles",
)

ax.legend(bbox_to_anchor=(1.0, 0.8), loc="upper right", fontsize=16)
# plt.legend()
plt.tight_layout()
plt.show()

odom["relative_time"] = odom["Time"] - odom["Time"].iloc[0]

laptime = 37.093  # fsg centerline
laptime = timestamp_max - timestamp  # fsg raceline

plt.plot(odom["relative_time"], odom["twist.twist.linear.x"], c="m", label="Circles")
plt.plot(
    odom_halfspaces["relative_time"],
    odom_halfspaces["twist.twist.linear.x"],
    c="g",
    label="Halfspaces",
)
plt.ylim(5, 10)
plt.xlim(0, 26.6)
# plt.xlim(0, 25.3)
# plt.xlim(0, laptime)
plt.xlabel("Time (s)", fontsize=20)
plt.ylabel("Velocity (m/s)", fontsize=20)
plt.yticks(fontsize=20)
plt.xticks(fontsize=20)
# plt.size = (10, 10)
plt.legend(fontsize=20)
plt.show()

# colour based on velocity
plt.plot(GT_left_boundary.T[0], GT_left_boundary.T[1], c="b")
plt.plot(GT_right_boundary.T[0], GT_right_boundary.T[1], c="y")
ax = plt.gca()
ax.axis("off")
ax.set_aspect("equal", "datalim")
plt.scatter(
    odom["pose.pose.position.x"],
    odom["pose.pose.position.y"],
    c=odom["twist.twist.linear.x"],
    cmap="viridis",
    s=4,
)
cbar = plt.colorbar()
cbar.set_label(label="Velocity (m/s)", fontsize=20)
cbar.ax.tick_params(labelsize=20)
plt.show()

# Plot steering
ms_steering = b.message_by_topic("/ugr/car/steering_velocity_controller/command")
steering = pd.read_csv(ms_steering)
print(steering.head())
steering = steering[steering["Time"] > timestamp]
steering = steering[steering["Time"] < timestamp_max]

steering["relative_time"] = steering["Time"] - steering["Time"].iloc[0]
plt.plot(steering["relative_time"], steering["data"], c="m", label="Circles")
plt.plot(
    steering_halfspaces["relative_time"],
    steering_halfspaces["data"],
    c="g",
    label="Halfspaces",
)
plt.xlabel("Time (s)", fontsize=20)
plt.ylabel("Steering rate (rad/s)", fontsize=20)
plt.yticks(fontsize=20)
plt.xticks(fontsize=20)
plt.xlim(0, 26.6)
# plt.xlim(0, 25.3)
# plt.xlim(0, laptime)
plt.ylim(-0.5, 0.5)
plt.legend(fontsize=20)
plt.show()

# transl_x = -20.626
# transl_y = -20.398
# # transl_y = -20.798
# yaw = 2.425

# timestamp = 57732.792
# 57736.213315143

# # Open the bag file using rosbag
# bag = rosbag.Bag('/home/zander/Documents/ugent_racing/rosbags/reference_tracking_halfspaces.bag')

# poses = []

# # Iterate through each message in the path topic
# for topic, msg, t in bag.read_messages(topics=['/mpc/ref_track']):
#     for pose in msg.poses:
#         pose_dict = {
#             'timestamp': t.to_sec(),
#             'position_x': pose.pose.position.x,
#             'position_y': pose.pose.position.y,
#             'position_z': pose.pose.position.z,
#             'orientation_x': pose.pose.orientation.x,
#             'orientation_y': pose.pose.orientation.y,
#             'orientation_z': pose.pose.orientation.z,
#             'orientation_w': pose.pose.orientation.w,
#         }
#         poses.append(pose_dict)

# bag.close()

# ref_poses_df = pd.DataFrame(poses)

# idx = (np.abs(ref_poses_df['timestamp'] - timestamp)).idxmin()
# print(ref_poses_df['timestamp'].iloc[idx])

# ref_poses_df_filtered = ref_poses_df[ref_poses_df['timestamp'] == ref_poses_df['timestamp'].iloc[idx]]

# # group all x and y positions by timestamp
# ref_poses_x = ref_poses_df_filtered.groupby('timestamp')["position_x"].apply(list)
# ref_poses_y = ref_poses_df_filtered.groupby('timestamp')["position_y"].apply(list)

# # Transform path from local frame to global frame
# ref_poses_x = np.array(ref_poses_x.iloc[0])
# ref_poses_y = np.array(ref_poses_y.iloc[0])
# ref_poses = np.array([ref_poses_x, ref_poses_y]).T


# rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
# ref_poses = np.dot(rot, ref_poses.T).T

# ref_poses_x = ref_poses.T[0] + transl_x
# ref_poses_y = ref_poses.T[1] + transl_y

# plt.plot(ref_poses_x, ref_poses_y, c='r')

# bag = rosbag.Bag('/home/zander/Documents/ugent_racing/rosbags/reference_tracking_halfspaces.bag')

# poses = []

# # Iterate through each message in the path topic
# for topic, msg, t in bag.read_messages(topics=['/mpc/x_prediction']):
#     for pose in msg.poses:
#         pose_dict = {
#             'timestamp': t.to_sec(),
#             'position_x': pose.pose.position.x,
#             'position_y': pose.pose.position.y,
#             'position_z': pose.pose.position.z,
#             'orientation_x': pose.pose.orientation.x,
#             'orientation_y': pose.pose.orientation.y,
#             'orientation_z': pose.pose.orientation.z,
#             'orientation_w': pose.pose.orientation.w,
#         }
#         poses.append(pose_dict)

# bag.close()

# pred_poses_df = pd.DataFrame(poses)

# idx = (np.abs(pred_poses_df['timestamp'] - timestamp)).idxmin()

# pred_poses_df_filtered = pred_poses_df[pred_poses_df['timestamp'] == pred_poses_df['timestamp'].iloc[idx]]

# # group all x and y positions by timestamp
# pred_poses_x = pred_poses_df_filtered.groupby('timestamp')["position_x"].apply(list)
# pred_poses_y = pred_poses_df_filtered.groupby('timestamp')["position_y"].apply(list)

# # Transform path from local frame to global frame
# pred_poses_x = np.array(pred_poses_x.iloc[0])
# pred_poses_y = np.array(pred_poses_y.iloc[0])
# pred_poses = np.array([pred_poses_x, pred_poses_y]).T

# rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
# pred_poses = np.dot(rot, pred_poses.T).T

# pred_poses_x = pred_poses.T[0] + transl_x
# pred_poses_y = pred_poses.T[1] + transl_y

# plt.plot(pred_poses_x, pred_poses_y, c='m')
# plt.plot(pred_poses_x[0], pred_poses_y[0], 'o', c='black')

# plt.xlim(-45, -25)
# plt.ylim(-21, 5)
# plt.show()
