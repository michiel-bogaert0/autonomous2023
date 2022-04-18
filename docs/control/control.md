# PID Car Control

This node controls the car by using two PID loops: one for the speed and another one for steering. Speed is held constant, the steering angle is determined with a simple algorithm which takes an exploration path as input.

## How steering works

First of all it tries to calculate the target angle is based on the exploration path. It uses the "carrot on a stick" principle. This means that the car tries to set its heading in such a way that the car drives to a target point (the carrot) which is always X meters ahead of the car and on the exploration path. See the picture below.

![image](https://user-images.githubusercontent.com/22452540/137592660-1fccbc5b-923d-422d-8609-ad523b9f41a0.png)

It calculates the target point in the following way:
- Convert the exploration path to a piecewise multi-segment equation with parameter t from `0` to `number_of_nodes(path)-1`.
- It iterates through the equation and calculates some heuristics
- When a point is found which meets the applied conditions, then this is the target point!

The system produces a lot more control messages than exploration path messages, but the target point will still be continously recalculated on the most recent exploration path and car odometry.

## Node parameters
Here is an overview of the parameters which can be used in this node.

| Parameter name | Default | Description |
|---|---|---|
| `/car_control/steering/Kp` | `2.0` | P gain of steering PID loop |
| `/car_control/steering/Ki` | `0.1` | I gain of steering PID loop |
| `/car_control/steering/Kd` | `0.2` | D gain of steering PID loop |
| `/car_control/speed/Kp` | `2.0` | P gain of speed PID loop |
| `/car_control/speed/Ki` | `0.0` | I gain of speed PID loop |
| `/car_control/speed/Kd` | `0.0` | D gain of speed PID loop |
| `/car_control/speed/target` | `3` | Speed target in `m/s` |
| `/car_control/publish_rate` | `1.0` | ROS publish rate of car control messages |
| `/car_control/stale_time` | `0.2` | Maximal age of an exploration path. When age becomes larges, the system turns 'stale', which causes emergency brake messages to be sent. When it receives an exploration path it once again becomes alive. |
| `/car_control/trajectory/minimal_distance` | `4` | the minimal 'stick distance', aka. the minimal distance between target point and car |
| `/car_control/trajectory/max_angle` | `1` | The maximal angle difference in rad between the the car and the target point |
| `/car_control/trajectory/t_step` | `0.05` | the timestep taken in the iteration process. |

