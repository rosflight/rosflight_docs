# Controller State Machine

## State Machine Overview

The controller state machine uses discrete states to turn on and off particular controls for a more stable flight.
These regimes are designed so that particular mission objectives are only activated when particular conditions are met.
In the default implementation this is controlled based on the aircraft's altitude.
Though, in practice these states could be effected by any input from a ROS topic.
The states are pictured below, and the differences in state are explored in the next section.

| ![Diagram of Controller State Machine](../../assets/Controller_state_machine.png "Controller State Machine") |
|:--:|
|*Figure 1: Controller state machine.*|

## States 

This section describes all of the states and what control is active during that phase of flight.

### Takeoff

In the takeoff state, the commanded airspeed is set to cruise, the commanded pitch is held constant and roll is controlled to zero.
This results in a steady takeoff directly along the runway.
The deactivation of course control means that the aircraft will not attempt to maneuver while too close to the ground.
The takeoff regime is only below a certain cutoff altitude.
After passing this threshold it moves into the climb state.

### Climb 

The climb state commands altitude and airspeed as normal, but still controls roll to zero.
This means that the aircraft will proceed directly ahead until it is withing a cutoff (see Params section) of commanded altitude.
Deactivating course control allows for the aircraft to gain enough altitude to be clear of all obstacles before attempting to follow the waypoint mission.
Once within a cutoff of the commanded altitude, the state transitions to altitude hold.
If the aircraft dips beneath the takeoff threshold, it will enter the takeoff state again.

### Altitude Hold

In altitude hold all control loops are closed normally. 
The aircraft will fly the waypoint mission as normal.
If the aircraft dips beneath the takeoff cutoff, the state transitions to takeoff again.
