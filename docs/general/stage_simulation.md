# Stage Simulation

The point of the `stage_simulation` package is to provide a normalized way of simulating stages like perception and mapping. This way, people can test their stages or new algorithms in an isolated, controlled environment.

# How to use

You can use the simulator by running a script in the scripts folder. You might need to run a launch file instead if you want to change the parameters or if you want to remap topics.

# Available simulations

In this section is an overview of all currently available simulation scripts and their supported parameters. Every stage simulation script supports a few global parameters. See the table below for an overview. 

Important to note is that `{name}` in the parameter name corresponds to the name of the simulation itself. See the specific descriptions below for the correct name

| Parameter name | Default | Description |
|---|---|---|
| `/stage_simulation/{name}/publish_rate` | `10` | ROS publishing rate of the simulation |

## Perception
- name: `perception`
- Publishes on:
  - `/processed/raw_perception_update`
- Subscribes to:
  - `/fsds/testing_only/odom`
  - `/fsds/testing_only/track`

This simulator tries to simulate the whole perception pipeline, including PnP. It is meant as input for mapping (aka. SLAM)

Here is a list of supported parameters

| Parameter name | Default | Description |
|---|---|---|
| `/stage_simulation/perception/fov` | `60` | The FoV used to decide which cones are 'visible' at the moment of publishing |
| `/stage_simulation/perception/add_noise` | `True` | Should noise be added to cone positions?
| `/stage_simulation/perception/cone_noise` | `0.2` | If noise is enabled, what is the magnitude?
| `/stage_simulation/perception/publish_delay` | `0.5` | The perception latency in seconds. This is used to test the latency between taking a picture and actually receiving a result from PnP.

## Odometry
- name: `odom`
- Publishes on:
  - `/odometry/filtered/global`
- Subscribes to:
  - `/fsds/testing_only/odom`

Used to simulate odometry data from the car. Supports no extra parameters

## Mapping
- name: `mapping`
- Publishes on:
  - `/processed/map/absolute`
- Subscribes to:
  - `/fsds/testing_only/track`

This simulator tries to simulate the whole mapping pipeline. The result is an absolute map of cones which should be coming from SLAM and can be used by path finding algoritmhs and control. Supports no extra parameters.