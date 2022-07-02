# General

The control plane of Localization and Mapping is responsible for controlling the modules inside Localization and Mapping. It basically maintains a local state of the system and acts upon changes. It can also be used for a few more advanced things like a Data Recovery System (DRS) in the case a sensor (or a few sensors) drop out. It also provides a generic interface to simulation, visualisation, logging and the global autonomous state.

<p align="center">
<img src="../img/locmap_control_plane.png" alt="ex1" width="70%" />
</p>

## Current implementation

Currently the control plane is basically a set of launch files. You can find them in `locmap_controller/launch`. There is one big launch file named `locmap_controller.launch`. You can use this one for most applications.


### `locmap_controller.launch` parameters

The big launch file has some parameters:

| Parameter name | Default | Value type | Description |
|---|---|---|---|
| `enable_sim_bridge` | `false` | `boolean` | Should `fsds_ros_bridge` be launched or not?
| `perception_mode` | `pipeline` | `pipeline | simulated | none` |  The system to use for perception.
| `slam_type` | `local_fastslam` | `perception_check | local_fastslam | global_clustering` | What SLAM configuration to use.
| `use_sim_time` | `true` | `boolean` | If set to `true`, it will use simulated time. Requires a publisher on `/clock`
| `use_interface` | `true` | `boolean` | Wether or not to use the LocMap interface, which can be used to convert data. For example from the FSDS to our own topics.

### Examples

Some example commands and applications:

| Command | Description
|---|---
| `roslaunch locmap_controller locmap_controller.launch perception_mode:=pipeline slam_type:=none use_interface:=true` | Use this to convert a rosbag captured from fsds (by only using `fsds_ros_bridge`) to something that can be used as input to a locmap-only simulation. It basically runs perception and the LocMap interface. Please see the notes below for some important remarks.
| `roslaunch locmap_controller fsds_simulation.launch perception_mode:=none slam_type:=perception_check use_interface:=false use_sim_time:=true` | Use this to sanity check the result of perception.

## Important notes regarding ROSBags

Be very careful around rosbags. See notes:

- When using some kind of sensor fusion or when you are relying on `tf` (so practically in all cases) be sure to play a rosbag with the `--clock` option and to set the global parameter `use_sim_time` to `true`.
- Be sure that when you play a rosbag with the `--clock` option and `use_sim_time:=true`, the rosbag **does not contain any messages on `/clock`**. Otherwise you'll have two publishers on the `/clock` topic, which causes massive problems.




## Future expansions

### Node management

Another important function of the control plane is to manage the framework in which LocMap works. So this means starting, stopping and restarting nodes, connecting them together by configuring topics, switching between different frameworks, etc...

### API for simulation, logging, global state, visualisation, etc...

To be determined...

### Data Recovery System (DRS)

Let's say that one of our sensors dropout, for example the GPS, and that sensor fusion notifies the control plane about this or that the control plane notices it himself by monitoring the topics. It can then configure the system in a way to keep this dropout in mind. It would for example tell Global SLAM that GPS is unreliable, so that it starts to buffer local SLAM output for the time being. When GPS comes back online, it can tell Global SLAM that GPS is fine again and that it must try to recover information from the buffered output.

In the case that a sensor dropout is critical (or becomes critical when it takes too long), it can also notify the modules to start the safety procedure.

### Loop Closure

Loop closure can be detected by multiple sources. The control plane can then pass on this information to the Global State or other modules which do not do this kind of detection.

