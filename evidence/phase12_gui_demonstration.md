
# Phase 12 Synchronized GUI Demonstration

## Scope

The video records a live three-UAV and one-Husky mission. UAVs perform sector
search and event-triggered digital-pheromone deposition. The Husky consumes the
shared pheromone map as its coordination input and performs target verification.
The visualization is read-only and publishes no robot control commands.

## Traceability

| Field | Value |
|---|---|
| Run ID | `run_20260715_175852_847453_event_triggered` |
| Private implementation revision | `3f526b5cc335ec5f23723ce287fdad1d89c12c8d` |
| Working tree at trial start | Clean |
| Communication mode | `event_triggered` |
| Status | `completed` |
| Termination | `all_targets_verified` |
| Trial valid | `true` |

## Outcome

| Metric | Value |
|---|---:|
| Verified targets | 3/3 |
| Canonical duration | 124.074 s |
| Mission completion | 122.430 s |
| Transmitted deposits | 954 |
| Policy-suppressed updates | 1,636 |
| Candidate suppression | 63.166% |
| Serialized payload | 26,712 bytes |
| Communication accounting valid | Yes |
| Forced kills / orphans | 0 / 0 |

The requested display hold was 300 s. RViz remained
available for 296.028 s before the presentation process exited.
Mission metrics and evidence validation were already finalized and explicitly
exclude this display-only interval.

## Video integrity

| Property | Value |
|---|---|
| File | `videos/phase12_event_triggered_uav_ugv_rviz_demo.mp4` |
| SHA-256 | `4025824715cd35b214fbe7c46dc6c9bfb0abc2fbf33b5826b712b322efc5268b` |
| Size | 10,373,954 bytes |
| Duration | 66.8431 s |
| Codec | H.264 |
| Resolution | 1920×1080 |
| Frame rate | 30000/1001 FPS |

## Interpretation

This is mechanism evidence, not a comparative result. The research claim is
reserved for replicated synchronized headless experiments with uncertainty
estimates and formal non-inferiority/equivalence analysis.
