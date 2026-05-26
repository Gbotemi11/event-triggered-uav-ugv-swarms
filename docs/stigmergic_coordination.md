# Stigmergic Coordination and Pheromone Evidence

This page gives public, non-confidential evidence that the simulation uses a pheromone-based stigmergic coordination loop.

## Verified Runtime Chain

1. DroneDepositor nodes publish pheromone deposit messages.
2. Deposit messages are sent to /pheromone_deposit.
3. PheromoneManager receives the deposit messages.
4. PheromoneManager updates the shared pheromone map.
5. The shared map is published on /pheromone_map.
6. HuskyForager reads /pheromone_map.
7. HuskyForager publishes UGV motion commands to /husky_0/cmd_vel.

## Runtime Topic Evidence

Pheromone deposit topic: /pheromone_deposit
Type: geometry_msgs/msg/Point
Publisher count: 3
Subscription count: 1

Sample live deposit message:
x: -0.037976525723934174
y: -0.006927582900971174
z: 1.0

## Pheromone Map Evidence

Pheromone map topic: /pheromone_map
Type: nav_msgs/msg/OccupancyGrid
Frame: map
Resolution: 0.5 m/cell
Width: 40
Height: 40
Total cells: 1600
Non-zero pheromone cells: 112
Minimum non-zero value: 1
Maximum pheromone value: 69

## Disclosure Note

This page provides high-level public evidence only. Full implementation details, complete source code, parameters, and manuscript-level derivations remain private while the work is being prepared for academic submission.
