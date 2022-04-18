# Generalized nodes

In order to easily build the framework, there are some generalized nodes which are extended by the actual implementations.

## SLAM node

This is a generic node that handles landmark based SLAM algorithms. There is a base class in `locmap_slam/src` which must be implemented per algorithm. The implementations for the current SLAM algorithms can be found in `locmap_slam/scripts`. An example of how to use them in a launch file can be found in the `locmap_controller/launch/slam/` folder.

The base class has some common parameters

### Generic parameters

| Parameter name | Default | Description |
|---|---|---|
| `world_frame` | `ugr/car_odom` | The "world" frame. Note that in this context "world" means what frame is deemed "fixed".
| `base_link_frame` | `ugr/car_base_link` | The base_link frame of the car (robot) to use for SLAM. 

### Input topics

| topic name| message type | description |
|---|---|---|
| `/input/observations` | `ugr_msgs/Observations` | A set of landmark observations to process |
| `/input/odometry` | `nav_msgs/Odometry` | The odometry used to localize the car. Should have the correct `frame_id` and `child_frame_id` according to the parameters provided. |

### Output topics

| topic name | message type | description |
|---|---|---|
| `/output/observations` | `ugr_msgs/Observations` | The map of landmarks, converted to "observations" relative to the base_link frame.
| `/output/map` | `ugr_msgs/Observations` | The map of landmarks, also converted to "observations", but now relative to the world frame.
| `/output/odom` | `nav_msgs/Odometry` | The predicted odometry of SLAM.