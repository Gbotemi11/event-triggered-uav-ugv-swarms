# System Architecture

This page gives a high-level, non-confidential view of the current UAV–UGV swarm coordination framework.

```mermaid
flowchart TD
    A[UAV Swarm] --> B[Event Trigger Logic]
    B --> C[Shared Digital Guidance Layer]
    C --> D[UGV Guidance Module]
    D --> E[UGV Navigation Behavior]

    F[Environment / Search Area] --> A
    F --> C
    E --> F

    G[Safety-Aware Coordination Layer] --> A
    G --> D

    H[ROS 2 Jazzy] --> A
    H --> D
    I[PX4 + Gazebo Simulation] --> A
    I --> D
```

## Description

The system uses a heterogeneous UAV–UGV team. UAVs observe the environment and update a shared digital guidance layer only when relevant event-trigger conditions are met.

The UGV does not receive direct commands from the UAVs. Instead, it uses the shared guidance field to support navigation decisions.

## Disclosure Note

This diagram is intentionally high-level. Full implementation details, source code, parameters, and manuscript-level derivations are currently private while the work is being prepared for academic submission.
