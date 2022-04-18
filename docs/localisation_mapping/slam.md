# General

This document gives an overview of all SLAM algorithms that we currently support are have developed.

# FastSLAM (1.0)

## Working principle

FastSLAM 1.0 is a landmark based SLAM algorithm from the EKF-SLAM family. It uses a Monte-Carlo particle filter in conjunction with a "point circular" motion model. This is a motion model which assumes that the robot drives in circular motion. This is of course not accurate in our case, but is good enough to start with.

In the picture below you can see the process of a typical single FastSLAM iteration. The basic principle is also described below:

- First comes the sampling step. Here all N existing particles are progressed through the motion model. It takes in velocity information from localization and also adds some noise.
- Per particle it does the following:
  - It tries to match the observations to the landmarks on the particle map (that is, each particle has its own landmark map and it tries to determine if a specific observation is new or if it can be linked to an existing landmark on said map)
    - If no match is found, it initializes a new landmark on the particle map (which is basically a new EKF)
    - If a match is found, the EKF of the landmark (and only from that landmark) is updated.
  - The particle map is filtered. The very bad landmarks are filtered out (aka. those that should be observed but were not observed for a few iterations ). The landmarks with a range that is too large are also filtered out (aka. local SLAM function)
  - Based on those observations a weight is calculated. In other words: how likely is this particle and his map?
- After all particles are processed the complete particle set is resampled based on their weights using [Fitness proportionate selection](https://en.wikipedia.org/wiki/Fitness_proportionate_selection)
- The output is the map corresponding with the particle that has the largest weight.

The time complexity of FastSLAM can be driven down to `N*log(M)`, where `N` is the amount of particles and `M` is the amount of landmarks in a particle map. The big improvement over EKF-SLAM is that the EKF filters are much smaller, so that the required matrix inversions are limited to 3x3 matrices.

Further reference: [FastSLAM paper](https://1drv.ms/b/s!Aure1n4O2j0E4CJ5utLtf-9O2IWw)

<p align="center">
<img src="../img/locmap_fastslam.png" alt="ex1" width="70%" />
</p>

## Implementation in ROS

The algorithm is implemented in the package `locmap_slam`, which extends the base SLAM node. It has support for Cython, so it is important to run `catkin build` every time you change something in this package.

## Example

Here you can see a small example of a simulation with FastSLAM. The blue line is Ground Truth, the red line is what SLAM thinks (referenced to the map frame) and the black line is what we would get without processing observations. The green stars are the estimates of the predicted landmarks, whilst the black landmarks are the GT landmarks. 

You can see that the map is only locally accurate (that is, from the perspective of the car iself). It is thus usable for Local SLAM.

<p align="center">
<img src="../img/locmap_fastslam_demo.gif" alt="ex1" width="70%" />
</p>
