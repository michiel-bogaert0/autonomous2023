# General

The control plane of Localization and Mapping is responsible for controlling the modules inside Localization and Mapping. It basically maintains a local state of the system and acts upon changes. It can also be used for a few more advanced things like a Data Recovery System (DRS) in the case a sensor (or a few sensors) drop out. It also provides a generic interface to simulation, visualisation, logging and the global autonomous state.

<p align="center">
<img src="../img/locmap_control_plane.png" alt="ex1" width="70%" />
</p>

## Current implementation

Currently the control plane is basically a set of launch files. You can find them in `locmap_controller/launch`.



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

