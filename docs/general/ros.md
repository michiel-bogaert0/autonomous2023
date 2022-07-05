# Topics

This is a full overview of all topics used by various subteams:

## Perception
| Topic name | Message | Description | Publish frequency (Hz) |
|---|---|---|---|
| /perception/raw_image | sensor_msgs/Image | Raw images coming from the camera  | 10 |
| /processed/cone_keypoints | ugr_msgs/ConeKeypoints | Keypoints after cone & keypoint detections | / |
| /processed/raw_perception_update | ugr_msgs/PerceptionUpdate | Cone locations | / |
|  |  |  |  |

# Coordinate frames & rotation

The simulator coordinates in the **ENU coordinate system** (x = east, y = north, z = up) where **x is pointing forward**.
All **rotations are right-handed**. Yaw is zero when pointing forward/east.

If possible, we stick to the above described coordinate system.
This system was chosen as it seems to be the the default for (autonomous) driving ([1](https://en.wikipedia.org/wiki/Axes_conventions), [2](https://www.mathworks.com/help/driving/ug/coordinate-systems.html)) and it is (the default for ROS)[https://www.ros.org/reps/rep-0103.html].

All data in global frame, that is all things not relative to the vehicle like position of the car odometry or cone positions, are relative to the starting position of the car.
For example, if the car spawned facing east and moved forward 1 meter and 0.5 meter right, it ends up at global position X=1;Y=-0.5;Z=0.

# Useful aliases
I use these aliases to make working with ROS easier:

```
alias ugr="cd /home/lucas/UGentRacing/autonomous/ROS && conda activate ros"
alias sdev="source devel/setup.zsh"
alias sros="source /opt/ros/noetic/setup.zsh"

source /opt/ros/noetic/setup.zsh # And some default sources
source /home/lucas/UGentRacing/autonomous/ROS/devel/setup.zsh
```

# Bugs
If ROS can't find some packages, make sure you have installed them. Not only on your system (apt install), but also in Conda (conda install) if they are used in Python. You can fix a lot of dependency problems using `rosdep install --from-paths src --ignore-src -r -y`.
