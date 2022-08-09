# Overview

## Purpose
This readme is designed to keep a log of my progress as I work through the real time embedded software case study. 

## Problem Overview
- see case study document...

## Problem Breakdown
- Identify/generate a potential state space model to model the Swoop Aero vehicle
- Generate test-data  to simulate the vehicle in cartesian coordinates during each landing phase
- Implement a visualiser in order to visualise the above [x,y,z] vehicle information
- Implement KF to estimate height of the vehicle relative to ground
	- May be able to get away with a linear KF since each of the landing phases is linear... ?
- Extend the model by addressing "bonus points" requirements
	- This seems to be an extension of the KF model to be non-linear domain... Possibly EKF?

Can visualise the problem by considering the following diagram from [3].
- This case study considers the model in the blue box. 
- Inputs to the model are the values from the vision system (i.e. lidar). 
- The output of the model will be vehicle's pose estimation / position in the cartesian grid...

![control loop diagram](./vision_based_control_diagram.png "Vision based control diagram") 

## Design Considerations
***placeholder***

### Assumptions

### Constraints

### System Environment

## Architectural Design
***placeholder***

### Component Diagram

### Deployment Diagram

## Low-Level Design

### Class Diagram

## References
[1] [State Space Representation](https://en.wikipedia.org/wiki/State-space_representation)
[2] [Full Linear Control of a Quadrotor UAV, LQ vs H∞](https://sci-hub.se/10.1109/control.2014.6915128)
[3] [Modeling and PD Control of a Quadrotor VTOL Vehicle](https://www.researchgate.net/publication/224719830)
