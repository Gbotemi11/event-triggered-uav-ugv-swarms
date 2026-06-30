#!/usr/bin/env python3
import csv
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MetricsLoggerResearch(Node):
    def __init__(self):
        super().__init__("metrics_logger_research")

        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path("~/swarm_ws/research_results").expanduser() / f"run_{stamp}"
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.events_path = self.run_dir / "events.jsonl"
        self.summary_path = self.run_dir / "summary.csv"

        self.start_time = time.time()

        self.detected_targets = set()
        self.verified_targets = set()
        self.depositor_stats = {}
        self.latest_map_metrics = {}

        self.event_sub = self.create_subscription(
            String, "/swarm/event", self.event_callback, 50
        )

        self.summary_file = open(self.summary_path, "w", newline="", encoding="utf-8")
        self.summary_writer = csv.DictWriter(
            self.summary_file,
            fieldnames=[
                "elapsed_sec",
                "detected_targets",
                "verified_targets",
                "total_deposits",
                "total_skipped",
                "overall_saving_pct",
                "active_cells",
                "mean_active_staleness_sec",
                "max_active_staleness_sec",
                "max_pheromone",
            ],
        )
        self.summary_writer.writeheader()

        self.create_timer(2.0, self.write_summary)

        self.get_logger().info(f"Metrics logging to: {self.run_dir}")

    def event_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        with open(self.events_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(data) + "\n")

        event_type = data.get("event_type")

        if event_type == "target_detected":
            self.detected_targets.add(int(data["target_id"]))

        elif event_type == "target_verified":
            self.verified_targets.add(int(data["target_id"]))

        elif event_type == "depositor_stats":
            drone_id = int(data["drone_id"])
            self.depositor_stats[drone_id] = data

        elif event_type == "map_metrics":
            self.latest_map_metrics = data

    def write_summary(self):
        total_deposits = sum(int(v.get("deposits", 0)) for v in self.depositor_stats.values())
        total_skipped = sum(int(v.get("skipped", 0)) for v in self.depositor_stats.values())
        total = total_deposits + total_skipped
        saving = (total_skipped / total * 100.0) if total > 0 else 0.0

        row = {
            "elapsed_sec": round(time.time() - self.start_time, 2),
            "detected_targets": len(self.detected_targets),
            "verified_targets": len(self.verified_targets),
            "total_deposits": total_deposits,
            "total_skipped": total_skipped,
            "overall_saving_pct": round(saving, 2),
            "active_cells": self.latest_map_metrics.get("active_cells", 0),
            "mean_active_staleness_sec": round(
                float(self.latest_map_metrics.get("mean_active_staleness_sec", 0.0)), 3
            ),
            "max_active_staleness_sec": round(
                float(self.latest_map_metrics.get("max_active_staleness_sec", 0.0)), 3
            ),
            "max_pheromone": round(float(self.latest_map_metrics.get("max_pheromone", 0.0)), 2),
        }

        self.summary_writer.writerow(row)
        self.summary_file.flush()

    def destroy_node(self):
        try:
            self.summary_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MetricsLoggerResearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
