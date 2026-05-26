# Running the UAV-UGV Swarm Simulation

This document gives a high-level, non-confidential guide for the current UAV-UGV swarm simulation demo.

## Simulation Environment

The current simulation uses:

- ROS 2 Jazzy
- PX4 SITL
- Gazebo
- Micro XRCE-DDS Agent
- QGroundControl
- Clearpath Husky UGV
- Multiple PX4 UAV agents
- Python-based swarm coordination scripts

## Main Paths

PX4-Autopilot: ~/PX4-Autopilot  
ROS 2 workspace: ~/px4_ros2_ws  
Swarm scripts: ~/swarm_ws/scripts  
Public project overview: ~/Desktop/event-triggered-uav-ugv-swarms  

## Launch Command

The full simulation can be launched with:

cd ~/swarm_ws/scripts  
./launch_all.sh  

The launch script starts PX4/Gazebo spawning, Micro XRCE-DDS Agent, QGroundControl, Husky bridge, pheromone manager, drone depositor, swarm takeoff, and the Husky forager terminal.

## Manual Husky Forager Start

If the Husky forager is not started automatically, run:

source /opt/ros/jazzy/setup.bash  
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  
source ~/px4_ros2_ws/install/setup.bash  
python3 ~/swarm_ws/scripts/husky_forager.py  

## Confirmed Sample Run

One observed simulation run produced:

| Metric | Value |
|---|---:|
| Task completion time | 10.80 seconds |
| Peak pheromone concentration | 83.0 |
| Distance travelled from spawn | 4.57 m |

The UGV published movement commands through /husky_0/cmd_vel and received odometry through /husky_0/odom.

## Public Evidence

The public repository includes:

- Gazebo setup screenshot
- UAV-UGV coordination screenshot
- short coordination demo video
- preliminary results summary
- system architecture overview

## Disclosure Note

This guide is intentionally high-level. It documents the workflow without exposing private code internals, manuscript derivations, or complete experiment parameters.
