# Exploration Path Planning

This node is responsible for calculating an exploration path between two lines of cones. The goal of exploration is to make the best map possible (for SLAM), so this path must ideally be in the middle of the track. The complexity in this node lies in the fact that it is not always clear what the sides of the track are. This is especially true when moving near sharp corners.

## How it works

The basic principle of the algorithm is as follows:
- Triangulate the space between the "cone map" coming from SLAM. It expects the absolute map. It also expects to receive odometry updates from mapping/localization.
- Calculate some heuristics
- Determine what the best path to take is (aka. the one that follows the centerline of the track)

## Node parameters
This is an overview of the supported node parameters

| Parameter name | Default | Description |
|---|---|---|
| `/exploration_mapping/max_iter` | `1000` | Maximal number of iterations before giving up. |
| `/exploration_mapping/plan_dist` | `12.0` | Maximal planning distance ahead of car |
| `/exploration_mapping/max_angle_change` | `0.8` | The maximal allowed angle change in radians. If exceeding this limit, the candidate path is thrown away. |
| `/exploration_mapping/safety_dist` | `1.0` | Determines how much space in `m` it should leave around the cones |
| `/exploration_mapping/expand_dist` | `1.0` | Not sure... |
| `/exploration_mapping/expand_angle` | `20.0` | Not sure... |
| `/exploration_mapping/use_branch_filter` | `False` | Not sure... |
