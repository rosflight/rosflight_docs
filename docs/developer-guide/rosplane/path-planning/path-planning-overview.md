# Path Planning Overview

The path planning methods used in ROSplane is modeled after the architecture presented in *Small Unmanned Aircraft: Theory and Practice* by Dr. Randal Beard and Dr. Tim McLain.

In their work, the path planning functionality of the aircraft is split into three main parts, as shown in Figure 1.

!!! note
    This is not a full description of the architecture.
    For example, the state estimator is not shown, and not all of the inputs to each of the blocks is included.
    These elements were removed from the figure for clarity.

| ![Diagram of Path Planning Architecture](../../../assets/path_planner_assets/path-planning-overview.svg "Path Planning Architecture") |
| :--: |
|*Figure 1: Path planning architecture - based on Figure 1.1 in* Small Unmanned Aircraft: Theory and Practice|

The high-level path planning is done by the *path planner* node.
The path planner's job is to manage, create, and publish waypoints.
Those waypoints are published to the *path manager* node.
The path manager's job is to determine and calculate the appropriate line segments and orbits needed to complete the waypoints.
The path manager then publishes these straight lines and waypoints to the *path follower*. 
The job of the path follower is to control the course, airspeed, and altitude of the aircraft to follow these lines and orbits.

For more information on each of these nodes, see the related documentation pages for the [path planner](./path-planner.md), the [path manager](./path-manager.md), and the [path follower](./path-follower.md).
See also chapters 10-13 of the *Small Unmanned Aircraft: Theory and Practice* book.

## Changing Path Planning
The path planner currently takes in a set of user-defined waypoints and follows those waypoints. 
This is not useful in all contexts, since in many applications the aircraft needs to take in sensor data and create its own waypoints and paths.

The modularity of this framework allows users to "plug in" their own algorithms for each of the blocks defined in Figure 1, instead of rewriting the whole path planning stack.

For example, we could replace the path planner block with a vision-based guidance block.
The vision-based-guidance block would make decisions based on visual data to create waypoints. 
However, the interface between the path planner (now the visual-based guidance planner) and the path manager would remain the same.

Chapter 13 of the *Small Unmanned Aircraft: Theory and Practice* book contains more detail on implementing a vision-based path planner.