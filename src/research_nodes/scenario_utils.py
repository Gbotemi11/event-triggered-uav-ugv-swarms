#!/usr/bin/env python3
import json
import os
from pathlib import Path

DEFAULT_SCENARIO = "~/swarm_ws/scenarios/disaster_industrial.json"

def load_scenario(path=None):
    scenario_path = Path(
        path or os.environ.get("SWARM_SCENARIO", DEFAULT_SCENARIO)
    ).expanduser()

    if not scenario_path.exists():
        raise FileNotFoundError(f"Scenario file not found: {scenario_path}")

    with open(scenario_path, "r", encoding="utf-8") as f:
        return json.load(f)

def get_uav_config(scenario, drone_id):
    for uav in scenario["robots"]["uavs"]:
        if int(uav["id"]) == int(drone_id):
            return uav
    raise ValueError(f"UAV id {drone_id} not found in scenario")

def get_sector_for_uav(scenario, drone_id):
    uav = get_uav_config(scenario, drone_id)
    return scenario["sectors"][uav["sector"]]
