# System Architecture

## Overview

The system implements an event-triggered producer-field-consumer architecture for heterogeneous UAV–UGV coordination.

UAVs act as aerial scouts and spatial-information producers. The pheromone manager maintains the shared spatial field. The Husky UGV consumes that field for navigation and performs close-range target verification. Verified regions generate clearing events that update the field.

---

## Main Components

### 1. Scenario Configuration

`config/disaster_industrial.json` defines:

- map properties
- robot configuration
- UAV sectors
- target locations
- experiment parameters
- field-staleness thresholds
- evaluation metrics

### 2. UAV Patrol

`src/research_nodes/swarm_patrol_research.py`

The patrol node assigns sector-based motion to the UAVs so that the aerial team searches different parts of the environment.

### 3. Event-Triggered Depositor

`src/research_nodes/drone_depositor_research.py`

The depositor evaluates candidate pheromone updates generated from UAV observations.

Each candidate is either:

- transmitted through `/pheromone_deposit`, or
- suppressed by the event-trigger policy.

Both outcomes are counted for communication-saving analysis.

### 4. Pheromone Manager

`src/research_nodes/pheromone_manager_research.py`

The manager maintains:

- pheromone concentration
- diffusion and decay
- active-cell count
- deposit count
- clearing count
- active-cell age
- mean staleness
- maximum staleness

It publishes:

- `/pheromone_map`
- `/pheromone_staleness_map`
- field metrics

### 5. Husky Field-Following Controller

`src/research_nodes/husky_forager_research.py`

The Husky UGV follows the pheromone gradient and publishes motion commands through `/husky_0/cmd_vel`.

The UGV is not directly commanded by an individual UAV. Its motion is generated from the shared field.

### 6. Target Verifier

`src/research_nodes/target_verifier_research.py`

The verifier determines when the UGV reaches a configured target region.

Successful verification publishes `/pheromone_clear`, allowing the associated field region to be removed or suppressed.

### 7. Metrics Logger

`src/research_nodes/metrics_logger_research.py`

The logger records:

- elapsed mission time
- detected targets
- verified targets
- transmitted deposits
- suppressed candidate updates
- overall update-saving percentage
- active pheromone cells
- mean field staleness
- maximum field staleness
- maximum pheromone concentration

---

## Data Flow

```text
UAV patrol and observation
          ↓
Candidate pheromone update
          ↓
Event-trigger evaluation
     ┌────┴────┐
     │         │
Transmit    Suppress
     │         │
     ↓         ↓
/pheromone_deposit   Saving counter
     │
     ↓
Pheromone manager
     ├── /pheromone_map
     ├── /pheromone_staleness_map
     └── field metrics
     │
     ↓
Husky field-following controller
     │
     ↓
Target verification
     │
     ↓
/pheromone_clear
     │
     └──────────────→ Pheromone manager
```

---

## Event-Trigger Metric

The implementation evaluates more candidate updates than it transmits.

```text
update saving (%) =
total_skipped
────────────────────────────── × 100
total_deposits + total_skipped
```

This is a message-update suppression metric.

It does not yet measure:

- transmitted bytes
- radio airtime
- packet loss
- energy consumption
- network congestion

---

## Distributed Spatial Memory Interpretation

The current implementation uses a shared ROS 2 field.

The research direction extends this toward robot-local spatial memories in which each robot stores and selectively exchanges persistent spatial information.

Future deposits may include:

- source robot
- spatial position
- semantic class
- confidence
- uncertainty
- novelty
- urgency
- creation time
- update time
- decay parameters
- verification status
- provenance

This would allow the system to study delayed, duplicated, stale, and contradictory field updates under intermittent communication.

---

## Experimental Scope

The public repository currently includes:

- integrated UAV–UGV mission summaries
- a 3, 6, 9, and 12-agent scalability experiment script
- a simulated UAV-failure experiment script

Only the integrated mission trials currently have retained public result summaries.

Controlled scalability and robustness datasets will be added before definitive numerical claims are published.
