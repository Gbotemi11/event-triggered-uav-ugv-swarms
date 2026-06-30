#!/usr/bin/env python3
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import String

from scenario_utils import load_scenario


class DroneDepositorResearch(Node):
    def __init__(self, uav_config, scenario):
        drone_id = int(uav_config["id"])
        super().__init__(f"drone_depositor_research_{drone_id}")

        self.scenario = scenario
        self.uav = uav_config
        self.drone_id = drone_id
        self.namespace = uav_config["namespace"]
        self.spawn_x = float(uav_config["spawn_x"])
        self.spawn_y = float(uav_config["spawn_y"])

        event_cfg = scenario["experiments"]["event_triggered"]
        baseline_cfg = scenario["experiments"]["continuous_baseline"]

        self.mode = os.environ.get("SWARM_MODE", event_cfg["mode"])
        self.min_move_distance = float(event_cfg["min_move_distance"])
        self.low_pheromone_threshold = float(event_cfg["low_pheromone_threshold"])
        self.target_event_strength = float(event_cfg["target_event_strength"])
        self.trail_strength = float(event_cfg["trail_strength"])
        self.deposit_period_sec = float(baseline_cfg["deposit_period_sec"])

        failure_cfg = scenario["experiments"]["failure_test"]
        self.failed_uav_id = int(os.environ.get("FAILED_UAV_ID", failure_cfg["failed_uav_id"]))
        self.failure_time = float(os.environ.get("FAILURE_TIME_SEC", failure_cfg["failure_time_sec"]))

        self.start_time = time.time()
        self.has_position = False
        self.local_x = 0.0
        self.local_y = 0.0
        self.world_x = self.spawn_x
        self.world_y = self.spawn_y

        self.last_deposit_x = None
        self.last_deposit_y = None
        self.last_periodic_deposit = 0.0
        self.last_target_deposit = {}

        self.pheromone_grid = None
        self.map_info = None

        self.verified_targets = set()

        self.total_updates = 0
        self.total_deposits = 0
        self.total_skipped = 0
        self.target_detections = 0
        self.trail_deposits = 0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        prefix = self.namespace

        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f"{prefix}/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            px4_qos,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/pheromone_map",
            self.map_callback,
            10,
        )
        self.verified_sub = self.create_subscription(
            String,
            "/target_verified",
            self.verified_callback,
            10,
        )

        self.deposit_pub = self.create_publisher(Point, "/pheromone_deposit", 10)
        self.event_pub = self.create_publisher(String, "/swarm/event", 10)

        self.create_timer(10.0, self.publish_stats)

        self.get_logger().info(
            f"Research depositor started for UAV {self.drone_id} | "
            f"mode={self.mode} | spawn=({self.spawn_x},{self.spawn_y})"
        )

    def publish_event(self, event_type, payload):
        msg = String()
        payload = dict(payload)
        payload["event_type"] = event_type
        payload["drone_id"] = self.drone_id
        payload["time_sec"] = time.time() - self.start_time
        msg.data = json.dumps(payload)
        self.event_pub.publish(msg)

    def map_callback(self, msg):
        self.pheromone_grid = np.array(msg.data).reshape(
            msg.info.height, msg.info.width
        ).astype(float)
        self.map_info = msg.info

    def verified_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.verified_targets.add(int(data["target_id"]))
        except Exception:
            return

    def get_pheromone_at(self, x, y):
        if self.pheromone_grid is None or self.map_info is None:
            return 0.0

        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        if 0 <= row < self.pheromone_grid.shape[0] and 0 <= col < self.pheromone_grid.shape[1]:
            return float(self.pheromone_grid[row, col])
        return 0.0

    def distance_since_last_deposit(self):
        if self.last_deposit_x is None:
            return float("inf")
        return math.hypot(
            self.world_x - self.last_deposit_x,
            self.world_y - self.last_deposit_y,
        )

    def is_failed(self):
        return (
            self.drone_id == self.failed_uav_id
            and (time.time() - self.start_time) >= self.failure_time
        )

    def publish_deposit(self, strength, reason, target=None, deposit_x=None, deposit_y=None):
        # Trail pheromones are deposited at the UAV position.
        # Target pheromones are deposited at the estimated target/inspection coordinate.
        x = self.world_x if deposit_x is None else float(deposit_x)
        y = self.world_y if deposit_y is None else float(deposit_y)

        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(strength)
        self.deposit_pub.publish(msg)

        self.last_deposit_x = self.world_x
        self.last_deposit_y = self.world_y
        self.total_deposits += 1

        if target is not None:
            self.target_detections += 1
            self.publish_event(
                "target_detected",
                {
                    "target_id": int(target["id"]),
                    "target_name": target["name"],
                    "target_type": target["type"],
                    "uav_x": self.world_x,
                    "uav_y": self.world_y,
                    "deposit_x": x,
                    "deposit_y": y,
                    "target_x": float(target["x"]),
                    "target_y": float(target["y"]),
                    "strength": strength,
                    "reason": reason,
                },
            )
        else:
            self.trail_deposits += 1

    def maybe_detect_target(self):
        now = time.time()
        cooldown = 6.0

        for target in self.scenario["targets"]:
            target_id = int(target["id"])
            if target_id in self.verified_targets:
                continue

            tx = float(target["x"])
            ty = float(target["y"])
            radius = float(target["detection_radius"])
            dist = math.hypot(self.world_x - tx, self.world_y - ty)

            if dist <= radius:
                last = self.last_target_deposit.get(target_id, 0.0)
                if now - last >= cooldown:
                    self.last_target_deposit[target_id] = now
                    strength = self.target_event_strength * float(target["priority"])

                    # Deposit the task pheromone at the target coordinate.
                    # This makes the pheromone field represent where the UGV should go,
                    # not merely where the UAV happened to be flying.
                    self.publish_deposit(
                        strength,
                        "target_in_detection_radius",
                        target,
                        deposit_x=tx,
                        deposit_y=ty,
                    )
                    return True

        return False

    def event_triggered_step(self):
        if self.maybe_detect_target():
            return

        dist = self.distance_since_last_deposit()
        local_conc = self.get_pheromone_at(self.world_x, self.world_y)

        if dist >= self.min_move_distance:
            self.publish_deposit(self.trail_strength, f"moved_{dist:.2f}m")
        elif local_conc < self.low_pheromone_threshold:
            self.publish_deposit(self.trail_strength, f"low_pheromone_{local_conc:.1f}")
        else:
            self.total_skipped += 1

    def continuous_step(self):
        now = time.time()

        if self.maybe_detect_target():
            return

        if now - self.last_periodic_deposit >= self.deposit_period_sec:
            self.last_periodic_deposit = now
            self.publish_deposit(self.trail_strength, "continuous_baseline")
        else:
            self.total_skipped += 1

    def position_callback(self, msg):
        self.total_updates += 1

        self.local_x = float(msg.x)
        self.local_y = float(msg.y)

        # PX4 local position is relative to each UAV spawn.
        # Convert it back into Gazebo/world coordinates.
        self.world_x = self.spawn_x + self.local_x
        self.world_y = self.spawn_y + self.local_y
        self.has_position = True

        if self.is_failed():
            self.total_skipped += 1
            return

        if self.mode == "continuous":
            self.continuous_step()
        else:
            self.event_triggered_step()

    def publish_stats(self):
        total = self.total_deposits + self.total_skipped
        saving = (self.total_skipped / total * 100.0) if total > 0 else 0.0

        self.get_logger().info(
            f"UAV {self.drone_id} stats | mode={self.mode} | "
            f"deposits={self.total_deposits}, skipped={self.total_skipped}, "
            f"saving={saving:.1f}%, target_detections={self.target_detections}"
        )

        self.publish_event(
            "depositor_stats",
            {
                "mode": self.mode,
                "deposits": self.total_deposits,
                "skipped": self.total_skipped,
                "saving_pct": saving,
                "target_detections": self.target_detections,
                "trail_deposits": self.trail_deposits,
                "failed": self.is_failed(),
            },
        )


def run_drone(uav_config, scenario):
    node = DroneDepositorResearch(uav_config, scenario)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()


def main():
    rclpy.init()
    scenario = load_scenario()

    threads = []
    for uav in scenario["robots"]["uavs"]:
        t = threading.Thread(target=run_drone, args=(uav, scenario), daemon=True)
        threads.append(t)
        t.start()

    print("Research drone depositors running. Press Ctrl+C to stop.")
    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        print("Shutting down research depositors.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
