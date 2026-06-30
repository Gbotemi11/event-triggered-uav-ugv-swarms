# Verified Preliminary Results

This document reports the public, non-confidential results retained from the current integrated UAV–UGV research deployment.

## Measurement Definition

The event-trigger policy evaluates candidate pheromone updates. A candidate update may be transmitted or suppressed.

```text
update saving (%) =
total suppressed candidate updates
──────────────────────────────────────── × 100
transmitted deposits + suppressed updates
```

This metric represents message-update suppression. It is not yet a measurement of packet bytes, radio airtime, energy use, or network throughput.

---

## Integrated Trials

| Run | Duration | Detected | Verified | Deposits | Suppressed | Saving |
|---|---:|---:|---:|---:|---:|---:|
| `run_20260625_200147` | 176 s | 3 | 2 | 1,639 | 23,801 | 93.56% |
| `run_20260625_200545` | 544 s | 3 | 2 | 4,356 | 76,536 | 94.62% |
| `run_20260625_201641` | 742 s | 3 | 2 | 5,445 | 105,501 | 95.09% |

All three complete runs:

- detected the three configured targets
- verified two targets
- produced event-triggered pheromone deposits
- suppressed most candidate updates
- logged field activity and staleness
- recorded pheromone-clearing behaviour

The achieved saving range was **93.56%–95.09%**.

---

## Latest Integrated Run

Run:

```text
run_20260625_201641
```

Final recorded values:

| Metric | Value |
|---|---:|
| Duration | 742 s |
| Targets detected | 3 |
| Targets verified | 2 |
| Deposits transmitted | 5,445 |
| Candidate updates suppressed | 105,501 |
| Overall update saving | 95.09% |
| Active cells | 545 |

The event stream also recorded field-clearing events during the mission.

---

## Interpretation

The retained trials establish that the integrated prototype can:

1. patrol with multiple aerial robots
2. detect configured mission targets
3. selectively deposit spatial information
4. maintain and decay a persistent pheromone field
5. guide a ground robot through the field
6. verify targets at ground level
7. clear verified pheromone regions
8. suppress a large proportion of candidate updates

These results demonstrate system feasibility. They do not yet establish statistical superiority over every alternative coordination method.

---

## Claims Not Yet Published

The repository contains implementations for scalability and UAV-failure experiments.

However, this release does not yet make definitive numerical claims for:

- performance at 12 agents
- quantified resilience after UAV removal
- packet-loss resilience
- communication latency
- transmitted bytes
- energy consumption
- statistical significance across repeated controlled trials

These require refreshed experiments with retained configurations, seeds, logs, and aggregate analysis.
