# Event-Triggered Stigmergic Coordination for Heterogeneous UAV–UGV Swarms

This repository presents a non-confidential research overview of an early-stage swarm robotics project focused on communication-efficient coordination for heterogeneous UAV–UGV teams.

The work explores how UAVs can support UGV navigation through indirect digital environmental signalling rather than continuous state broadcasting. The goal is to reduce communication load while preserving useful coordination, scalability, robustness, and safety-aware navigation.

## Research Direction

The project investigates communication-efficient swarm robotics for heterogeneous robot teams operating under limited communication. It combines ideas from:

- swarm robotics
- stigmergic coordination
- event-triggered communication
- UAV–UGV cooperation
- safety-aware multi-agent autonomy
- ROS 2, PX4, and Gazebo simulation

## Current Status

The system has been implemented in simulation using ROS 2 Jazzy, PX4, and Gazebo with a heterogeneous UAV–UGV setup.

Current experiments include:

- communication reduction compared with continuous broadcasting
- UAV–UGV guidance through an indirect shared guidance layer
- scalability testing with larger agent teams
- robustness evaluation under agent failure
- safety-aware coordination using control barrier function concepts

The full manuscript is currently in preparation and has not yet been publicly released.

## Related UGV Platform Work

In parallel, I am building an edge-device-based custom UGV platform for visual-inertial sensing and future autonomous navigation research. The current platform uses two CSI cameras and an IMU connected through a USB serial bridge, running on ROS 2 Jazzy.

Current progress includes:

- calibrated IMU publishing to `/imu/data`
- dual-camera ROS topic publishing
- synchronized ROS bag recording
- repeatable health-check, logging, calibration, and startup scripts
- preparation for visual and visual-inertial SLAM evaluation

## Planned Extensions

Future work includes:

- stronger theoretical analysis of event-triggered swarm coordination
- additional baseline comparisons
- simulation video release
- hardware-oriented validation
- semantic and dynamic SLAM for UGV navigation
- preparation for conference submission

## Note

This repository is a public research overview. Full implementation details, code, and manuscript are currently private while the work is being prepared for academic submission.

## Contact

**Oluwagbotemi Elijah Ogundipe**  
B.Eng. Mechatronics Engineering  
Email: oluwagbotemi6ogundipe@gmail.com
