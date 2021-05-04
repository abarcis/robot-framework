This repository contains the framework for testing different swarm behaviors in
simulation and on different robotic platforms.

The files `main_xxxxxx.py` define different configurations for specific
experiments. For the most recent examples see: `main_kpkdemo_px4.py` or
`main_discrete_sync_and_swarm.py`.

# Key components

## Controller
Runs the main loop of the program. Supports single or multiple agents, depending
on if the experiment is centralized or distributed.
Currently implemented controllers:
* `OfflineController` --- centralized controller used mainly for simulations.
* `SynchronizedOfflineController` --- centralized controller for experiments
with synchronous algorithms.
* `ROSController` --- controller for experiments using ROS2.

## Logic
Implementation of different swarm behaviors.

## Visualization
Abstraction for visualization of swarm behavior. The control of robots is also
treated as visualization. Currently implemented:
* `LiveVisualization` displays the state of agents in matplotlib.
* `BalboaVisualization` controls Balboa self-balancing robots.
* `PX4Visualization` controls PX4-based drones.
* `Crazyflie` controls Crazyflie drones (special case that is also an
implementation of position feedback)

## Position feedback
Feedback about the position of agents. Currently implemented:
* `PositionFeedback` --- basic simulator that takes the current position and
velocity and outputs new position.
* `Optitrack` --- reads positions received from the Optitrack mocap system over
ROS2.
* `PX4` --- reads positions received from PX4 over ROS2.
* `Crazyflie` --- reads positions provided by crazyswarm.

## Knowledge
Defines how the agents keep their knowledge. There are two options:
`SharedKnowledge` and `SeparateKnowledge`.

## Communication
Defines how agents communicate with each other. Currently implemented:
* `OfflineCommunication` for communication based on shared memory.
* `OfflineDistributedCommunication` for communication based on shared memory but
with separated knowledge (possible filtering of messages, not only broadcast).
* `ROSCommunication` for communication based on ROS2.
