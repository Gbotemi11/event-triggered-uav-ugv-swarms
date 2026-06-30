#!/usr/bin/env python3
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import String

from scenario_utils import load_scenario


class PheromoneManagerResearch(Node):
    def __init__(self):
        super().__init__("pheromone_manager_research")

        self.scenario = load_scenario()
        m = self.scenario["map"]

        self.origin_x = float(m["origin_x"])
        self.origin_y = float(m["origin_y"])
        self.size_x = float(m["size_x"])
        self.size_y = float(m["size_y"])
        self.resolution = float(m["resolution"])
        self.publish_hz = float(m["publish_hz"])
        self.evaporation_rate = float(m["evaporation_rate"])
        self.diffusion_rate = float(m["diffusion_rate"])

        self.width = int(self.size_x / self.resolution)
        self.height = int(self.size_y / self.resolution)

        self.grid = np.zeros((self.height, self.width), dtype=np.float32)

        self.start_time = time.time()
        stale_threshold = float(self.scenario["metrics"]["staleness_threshold_sec"])
        self.last_update = np.full(
            (self.height, self.width),
            self.start_time - stale_threshold,
            dtype=np.float64,
        )

        self.deposit_count = 0
        self.clear_count = 0

        self.map_pub = self.create_publisher(OccupancyGrid, "/pheromone_map", 10)
        self.stale_pub = self.create_publisher(OccupancyGrid, "/pheromone_staleness_map", 10)
        self.event_pub = self.create_publisher(String, "/swarm/event", 10)

        self.deposit_sub = self.create_subscription(
            Point, "/pheromone_deposit", self.deposit_callback, 10
        )
        self.clear_sub = self.create_subscription(
            Point, "/pheromone_clear", self.clear_callback, 10
        )

        self.create_timer(1.0 / self.publish_hz, self.update_and_publish)
        self.create_timer(1.0, self.publish_metrics)

        self.get_logger().info(
            f"Research pheromone manager started: "
            f"{self.size_x}m x {self.size_y}m, "
            f"{self.width}x{self.height} cells, res={self.resolution}m"
        )

    def world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def grid_to_world(self, row, col):
        x = self.origin_x + col * self.resolution
        y = self.origin_y + row * self.resolution
        return x, y

    def publish_event(self, event_type, payload):
        msg = String()
        payload = dict(payload)
        payload["event_type"] = event_type
        payload["time_sec"] = time.time() - self.start_time
        msg.data = json.dumps(payload)
        self.event_pub.publish(msg)

    def add_gaussian(self, x, y, strength, sigma):
        cell = self.world_to_grid(x, y)
        if cell is None:
            return False

        row, col = cell
        radius_cells = max(1, int((3.0 * sigma) / self.resolution))
        now = time.time()

        for rr in range(row - radius_cells, row + radius_cells + 1):
            for cc in range(col - radius_cells, col + radius_cells + 1):
                if 0 <= rr < self.height and 0 <= cc < self.width:
                    wx, wy = self.grid_to_world(rr, cc)
                    d2 = (wx - x) ** 2 + (wy - y) ** 2
                    value = strength * math.exp(-d2 / (2.0 * sigma * sigma))
                    if value > 0.05:
                        self.grid[rr, cc] += value
                        self.last_update[rr, cc] = now

        np.clip(self.grid, 0.0, 100.0, out=self.grid)
        return True

    def deposit_callback(self, msg):
        strength = float(msg.z)
        sigma = 1.4 if strength >= 20.0 else 0.8

        ok = self.add_gaussian(float(msg.x), float(msg.y), strength, sigma)
        if ok:
            self.deposit_count += 1

    def clear_callback(self, msg):
        x = float(msg.x)
        y = float(msg.y)
        radius = float(msg.z) if msg.z > 0.0 else 2.0

        cell = self.world_to_grid(x, y)
        if cell is None:
            return

        row, col = cell
        radius_cells = max(1, int(radius / self.resolution))
        now = time.time()

        for rr in range(row - radius_cells, row + radius_cells + 1):
            for cc in range(col - radius_cells, col + radius_cells + 1):
                if 0 <= rr < self.height and 0 <= cc < self.width:
                    wx, wy = self.grid_to_world(rr, cc)
                    if math.hypot(wx - x, wy - y) <= radius:
                        self.grid[rr, cc] = 0.0
                        self.last_update[rr, cc] = now

        self.clear_count += 1
        self.publish_event("pheromone_cleared", {"x": x, "y": y, "radius": radius})

    def update_and_publish(self):
        if self.diffusion_rate > 0.0:
            neighbours = (
                np.roll(self.grid, 1, axis=0)
                + np.roll(self.grid, -1, axis=0)
                + np.roll(self.grid, 1, axis=1)
                + np.roll(self.grid, -1, axis=1)
            ) / 4.0
            self.grid = (1.0 - self.diffusion_rate) * self.grid + self.diffusion_rate * neighbours

        self.grid *= self.evaporation_rate
        self.grid[self.grid < 0.05] = 0.0

        self.publish_grid(self.grid, self.map_pub, "map")

        now = time.time()
        threshold = float(self.scenario["metrics"]["staleness_threshold_sec"])
        age = np.clip((now - self.last_update) / threshold * 100.0, 0.0, 100.0)
        self.publish_grid(age, self.stale_pub, "map")

    def publish_grid(self, array, publisher, frame_id):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = np.clip(array, 0, 100).astype(np.int8).flatten().tolist()
        publisher.publish(msg)

    def publish_metrics(self):
        now = time.time()
        active = self.grid > 1.0

        if np.any(active):
            active_age = now - self.last_update[active]
            mean_staleness = float(np.mean(active_age))
            max_staleness = float(np.max(active_age))
            active_cells = int(np.sum(active))
        else:
            mean_staleness = 0.0
            max_staleness = 0.0
            active_cells = 0

        self.publish_event(
            "map_metrics",
            {
                "active_cells": active_cells,
                "mean_active_staleness_sec": mean_staleness,
                "max_active_staleness_sec": max_staleness,
                "deposits_received": self.deposit_count,
                "clears_received": self.clear_count,
                "max_pheromone": float(np.max(self.grid)),
            },
        )


def main():
    rclpy.init()
    node = PheromoneManagerResearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
