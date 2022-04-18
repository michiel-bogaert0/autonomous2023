# Transformations

## Introduction

One of the big issues in the system is that data is not synchronized and possibly delayed. Between perception and mapping there is for example a time delay between the moment a certain image was taken and when mapping receives the data from that image. The car is moving at a non-constant speed and the locations are given relative to for example the car frame, so that means we introduce a lot of extra error. For example, when the car is moving at `1m/s` (which is very slow) and the perception latency of that image is `300ms`, the mapping algo will already place the cones wrong by about `30cm`. Not that big of a deal for straights, but it causes the car to overestimate the radius of corners, which causes understeering. Due to the fact the car moves at a non-constant speed, the system does not have a good way to actually correct this by itself. 

This is why the whole system (not only mapping) makes use of the `tf2` package. This package allows us to introduce coordinate frames and to keep track of transformations between those frames while also taking time in account. 

Please read the [wiki](http://wiki.ros.org/tf2) for more information about this package, but here are the core concepts.

## The /tf and /tf_static topics

The tf2 package requires you to introduce coordinate frames, which are basically strings formatted in a certain way. Then you can broadcast transformations between frames on any given time. Such a transformation is basically a message of type [`geometry_msgs/TransformStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TransformStamped.html). 

These messages are then published on the `/tf` or `/tf_static` topics depending on the type of transformation. A static transformation is just one that does not change in time. Messages on this topic are only published once, but are also latched. This means that a node subscribing to this topic will still receive all static transformations (once).

Do realize that every node that wishes to use this package is in normal operation obligated to subscribe to these topics! The tf2 package does this automatically, but this has some performance issues, see below

## The `std_msgs/Header` message

This is a very important message type which you can include in other message types (such as geometry messages or perception updates). It basically contains a ROS timestamp and a frame id. This frame id must correspond with a coordinate frame. The header basically means that this message was produced in frame `frame_id` on timestamp `timestamp`. This allows the node to know from what (or to what) frame it must transform, and more important, at which time in the past it must transform.

It is best practise to introduce this in all message types if relevant.

## Introduced frames

We follow the [REP 105](https://www.ros.org/reps/rep-0105.html) convention for coordinate frames and frame authorities. Make sure to read that convention! Also take a look at the [Localization and Mapping](/localisation_mapping/) documentation for a better overview about our frame conventions.

## Performance optimization

Normally every node is responsible for keeping track of all transformation. This has a few issues: 
- It introduces a lot of unnecessary memory and network usage as every node must buffer these transformation.
- When your node is resource-intensive, it might miss some transformation or receive them way too late, which could introduce some transformation inaccuracies and/or problems.

This is why in our repository there is also a package based on [tf_service](https://github.com/magazino/tf_service). This package allows us to centralize all transformation information and to provide a transformation service. This way only one node is responsible for keeping track of transformations and transforming data happens by calling a service. The biggest downside is that it could introduce some extra latency due to the fact it must wait on a service call in order to transform messages, but it could increase performance by letting non-critical nodes use this system.

Check the GitHub Repository for information about how to use the system.

## Transformations of custom message types

Normally speaking `tf2` only allows to transform geometry messages. When we receive for example cone locations, we must then normally first convert it to a list of geometry messages, transform them each and then reinsert them in the perception update. This causes a lot of overhead. It is much better (and relatively simple) to provide custom transformations for custom message types.

How you should do this is badly documented. You can take a look at `node_fixture.py` to figure out how to do this. There are already implementations for the message types `Map` (`do_transform_map`) and `PerceptionUpdate` (`do_transform_perception_update`). Using these functions is relatively easy. Just call them with your message and a transformation, which you got from calling the `lookup_transform` or `lookup_transform_full` functions from `tf2`. You could also supply a generic (custom made) `TransformStamped` message because a transformation coming from `tf2` is basically a message with that type. 
