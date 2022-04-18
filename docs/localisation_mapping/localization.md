# General

Localization is the act of "localizing the robot" in an actual, physical environment. It can be combined with mapping, which gives you SLAM. At the moment we are splitting the localization process into two parts: sensor fusion and SLAM. 

Sensor fusion tries to localize the car based on multiple sensor measurements of the standard 15-dimensional robot state. In other words, it doesn't really care about obstacles in the physical environment, only on how it moves through it. Supported sensors include IMU's, gyroscopes, ground speed sensors, wheel encoders, GPS, virtual measurements in some kind of feedback loop coming from post-SLAM systems, etc... All these sensors must be 'fused', that is, multiple measurements of the same motion parameter (such as forward acceleration) must be 'averaged' together into one single measurement, which can then be used to finally calculate position estimates. 

The problem with sensor fusion is that the estimates tend to drift, even when you have a very accurate description of the behaviour of the car (aka. the motion model).

SLAM on the other hand, tries to localize the car primarily based on the environment itself. The environment is typically unkown at first, so at the same time it also tries to map it. This is a very difficult issue! But, when done correctly it can provide you with much better localization estimates without having to have a super detailed motion model. 

# Frame conventions

The first thing to do before diving into localization is to take a look at the frame conventions we use, which also follows [REP105](https://www.ros.org/reps/rep-0105.html).

First of all, we have the earth frame, which acts as the 'root'. This frame is our global frame of reference, from which we assume it never changes. We are not doing space travel, so this assumption is fine. 

<p align="center">
<img src="../img/frame_conventions.png" alt="ex1" width="60%" />
</p>

Then, **per car**, we have a string of specific frames. The first one is the **map** frame. This is the frame that can be used to fetch position estimates, but not for speed / acceleration estimates. In general the map frame could be moving relative to earth, but since our map (aka. the racetrack) is small enough, this is not the case for us and the map frame is statically fixed to earth. The map frame is also used as a reference for the racetrack and the **global** path planning (aka. raceline calculations).

Attached to the map frame is the **odom** frame. This frame can be used as a reference when in need of local, continuous estimates like speed and/or acceleration. Don't use this frame for absolute positioning! In a perfect world, this frame would be identical to the map frame, but of course the world is not perfect. Thusthe system calculates a transformation between them, for example based on GPS information. It basically corrects the drift of **odom** (see further below).

And finally we have the base link frame. The base link should attached to the center of gravity of the car. From there you should attach new frames, one per sensor. Every sensor message should then have the correct frame id, so that the localization module can correctly transform it to the base_link frame before fusing. This is to accomodate for the fact that two sensors might make different measurements based on their actual physical mounting positions, which can be 'corrected' using the sensor frame to base link frame transformations. The transformation can also be set dynamically to for example correct errors in sensor mounting.

# Sensor fusion

We use the `robot_localization` package to apply sensor fusion to our car. The package follows the REP105 and REP103 conventions, which we also use. It also uses an onmidirectional motion model, which is not perfect for our application, but good enough to start.

Please take a look at [the `robot_localization` wiki](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) and [this video](https://www.youtube.com/watch?v=nfvvpYBAMww) for more information about the package. Those together with this documentation and the example launch file (`locmap/template.sensor_fusion.launch`) should get you started, but here is an overview anyways.

<p align="center">
<img src="../img/loc_sensor_fusion.png" alt="ex1" width="60%" />
</p>

## Sensor frames and transformations

First of all, it is **very important** to ensure that all incoming sensor messages have a `frame_id` for which a (static) transformation to the `base_link` exists. Take for example an IMU. The message that localization expects from an IMU is `sensor_msgs/Imu`, which contains a header, which in turn contains a timestamp and a `frame_id`, which could be one of those extra sensor frames based on `base_link`. It needs to know the IMU data relative to the `base_link` frame, so therefore a (static) transformation between these two frames must exist. 

In the template launch file it is just a static identity transformation. This does not have to be the case though. For IMU's it can be useful to set this transformation to something on the start of the system to correct for example wrong mounts (where for example yaw does not correspond with rotation around the Z-axis, as the system expects)

## ekf_localization_node_odom_{carname}

This node is responsible for sensor fusion with the transformation between the **odom** frame and the **base_link** frame as a result. The transformations are published to `/tf`. The node also publishes the resulting odometry messages to `{carname}/odometry/filtered/odom`. These messages represent the pose of the car, relative to the (only in this context 'world-fixed') odom frame. In other words, you must interpret them in such a way that the odom frame is fixed, that there is zero drift and that these messages then represent the 'actual' car pose.

But the truth is that the coordinates of the base link relative to the odom frame may drift over time when interpreted relative to the actual world-fixed map (or earth) frame. **So only use the output of this node for stuff like velocity, where a drift does not matter that much**. It is however always continuous.

**Note:** do not fuse absolute sensor data like GPS position! This is done in another node! Only stuff like IMU, wheel encoders, GSS are allowed...

## ekf_localization_node_map_{carname}

This node is very similar to the previous node, but it can also take absolute sensor data like GPS. It outputs transformations between the **map** frame and the **odom** frame. It first estimates the transformation between **map** and **base_link**. Then it calculates the transformation between **map** and **odom** by looking up the transformation between **base_link** and **odom**. Please realise that the last transformation (**base_link** and **odom**) comes directly from the previous node!

This may seem a bit confusing (why publish transformations between **odom** and **map** instead of directly publishing transformations between **map** and **base_link**), but the reason is that in ROS a frame can only have a single parent (and the parent of **base_link** is **odom** by convention). Mathematically this doesn't change a lot, but it may introduce some small instabillities which causes the robot to jitter a bit while standing still.  

The GPS data itself (which is also an Odometry sensor) comes from the third and final node (see further)

Next to transformations it also publishes the corresponding odometry messages to `{carname}/odometry/filtered/map`. The messages coming on this topic may contain jumps, coming from the fact that it gets constantly fixed with absolute data (like GPS). These odometry messages are relative to the world-fixed map frame. So they represent the absolute, globally 'correct' robot pose. **So only use this for global positioning/heading data, not for velocity/acceleration**

## navsat_transform_node_{carname}

This node transforms a `sensor_msgs/NavSatFix` message (which usually comes from the GPS and contains stuff like longitude, which we cannot use) to an odometry message referenced to the correct frame.

Next to GPS fixes it also requires two other inputs:

- IMU data (**with heading information!!**)
- odometry messages (**use the output from the previous node, `{carname}/odometry/filtered/map`**)

It then publishes the odometry gps data to `{carname}/odometry/filtered/gps`. Those messages can then be used in the previous node.

At first sight it may seem like there is some kind of weird circular refencing with the previous node, but this is not a problem. 

## How to run

Take a look at the launch files inside the `locmap_controller` package. You can run them using `roslaunch`.

## Example

To conclude sensor fusion, we are going to take a look at a practical example in the form of a GIF. The blue line is the ground truth path. The coordinate frames are the frames discussed earlier. Note that the accuracy is way off and drifts very quickly, but this is by design. The simulation had only one single IMU and a very slow GPS (about 2FPS). This allows us to really understand what the limits of this system are.

The first GIF is from the perspective of earth (so used in absolute positioning). The second GIF is with respect to the odom frame (so used for speed/acceleration estimates)

<p align="center">
<img src="../img/loc_sensor_fusion_example.gif" alt="ex1" width="60%" />
</p>
<p align="center">
<img src="../img/loc_sensor_fusion_example_2.gif" alt="ex2" center width="60%"  />
</p>

A few interesting things:

- In the first GIF the base_link frame jumps around (which is because of GPS fixes coming in) while in the second GIF it stays one smooth continous path
- The position error gets worse when the velocity is higher. Also when the car is standing still in the end the odom frame starts drifting backwards with a contant velocity. This is due to the fact that in this simulation we have only used IMU data. When estimating position it must integrate the accelerations coming from IMU twice. So you basically add two kinds of errors. One absolute position error (which is not that bad, can be fixed with GPS) and another constant velocity error (which is very bad). This is exactly the reason why it is much better to also fuse wheel encoders or ground speed sensors, as we can then eliminate said velocity error.

# SLAM (global overview)

The second way of localization is SLAM. We are not going into details about different SLAM algoritms because there are a lot of different methods, each with their own pros and cons. Instead, here you'll find a generic framework that explains the basic concepts behind how we integrate SLAM with the rest of the system.

SLAM stands for Simultanious Localization and Mapping, which as the name itself implies, tries to localize the robot in an at first unkown environment, which it also tries to map at the same time. The localization component of SLAM is (in general) quite similar to sensor fusion in the sense that it also uses a motion model in conjunction with some kind of filter. The main difference lies in what information is being 'fused'. In sensor fusion you only fuse information that directly measure a substate of the robot pose, while in SLAM the localization part also fuses measurements of the environment.

<p align="center">
<img src="../img/slam_global_overview.png" alt="ex1" width="70%" />
</p>

In the picture above you can see an example implementation of how SLAM is integrated in the system. Now we will talk a bit about each module

## Local SLAM

The local SLAM module is probably the most important one. There are a lot of different SLAM methods, but it must be landmark based. So as its observation input it must take discrete range/bearing measurements that correspond to the landmarks as seen by the robot in that time instance. Even when working with LIDAR (which has their own complete SLAM families) must be first converted to range/bearing measurements.

There are multiple reasons for this. The primary one is that it is easier to implement, because we only have to deal with a single SLAM framework (aka. a landmark based one) instead of two. It also supports all kinds of 'environment sensors', like cameras (stereo or mono), LIDARs, depth sensors, etc... Another reason is the fact that we absolutely must do this in realtime, and the methods specifically built for LIDAR data are usually not that efficient. And finally it makes more sense, because a track is a fairly simple environment with landmarks which can be modelled very well as discrete 'points'. Of course this design choice must be evaluated in the future.

Local SLAM generally also takes in speed estimates (which may come from sensor fusion) to feed to its internal motion model.

Note that local SLAM is situated in the odom frame, which means that it works with continous data that might drift over time. It can correct some of this, but because of other issues like heavy linearizations the map that it produces is in general not usable as a globally accurate map estimate. Instead it produces a local map estimate with a limited range. That is, if landmarks are farther than X meters, they will get removed from the map. Think of local SLAM as some kind of buffer. It takes in for example 6 raw (inaccurate) landmark observations at a time, but manages information about 30 landmarks, wich are in itself much more accurate, due to multiple observations of the same landmarks.

## Global SLAM / Mapping

Of course we must have a globally accurate map in order to do global path planning. This is what this module is responsible for. It takes in position estimates (coming from the sensor fusion) and the local map estimates from local SLAM. It tries to build a global map based on that.

The reason why we split SLAM into a local and a global part is very similar to why sensor fusion does this. Suppose that we use 'local SLAM' referenced to the map frame instead of to the odom frame. In the case of a good GPS signal, the produced map will be globally accurate. But, in the case of a bad GPS signal (even when only for a few seconds) the produced map will be sligthly off. Also, the position estimates might jump around due to the GPS corrections every once in a while. SLAM doesn't really like these jumps. 
 
Another way to think about this module is as a module which assumes very accurate localization (aka. GPS) and then works with very accurate sensors which produce 30 accurate landmark observations at once, instead of the 6 inaccurate ones our actual sensors would produce. The assumption of accurate localization also boosts the performance.

The module can also do some more advanced things, like buffering local SLAM output in the event that our GPS drops out, detect loop closures, etc...

## A note about motion models

At the moment our local SLAM and sensor fusion uses their own (simple) motion model. In the future we should think about a general motion model which can be exposed as a service to SLAM, sensor fusion, path planning and control.

