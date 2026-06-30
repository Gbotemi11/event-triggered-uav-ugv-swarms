#!/usr/bin/env python3

import json
import math
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String

from scenario_utils import load_scenario


class TargetVerifierResearch(Node):
    """
    Simulation-only target verifier.

    The Husky controller does not receive target coordinates.
    This node uses scenario ground truth only to evaluate whether a
    reported pheromone peak corresponds to a real target.
    """

    def __init__(self):
        super().__init__("target_verifier_research")

        self.scenario = load_scenario()
        self.targets = self.scenario["targets"]
        self.verified_targets = set()
        self.start_time = time.time()

        self.peak_sub = self.create_subscription(
            String,
            "/pheromone_peak_reached",
            self.peak_callback,
            10,
        )

        self.verified_pub = self.create_publisher(
            String,
            "/target_verified",
            10,
        )

        self.clear_pub = self.create_publisher(
            Point,
            "/pheromone_clear",
            10,
        )

        self.event_pub = self.create_publisher(
            String,
            "/swarm/event",
            10,
        )

        self.get_logger().info(
            "Simulation ground-truth target verifier started"
        )

    def publish_event(self, event_type, payload):
        message = String()

        data = dict(payload)
        data["event_type"] = event_type
        data["time_sec"] = time.time() - self.start_time

        message.data = json.dumps(data)
        self.event_pub.publish(message)

    def peak_callback(self, msg):
        try:
            data = json.loads(msg.data)
            husky_x = float(data["husky_x"])
            husky_y = float(data["husky_y"])
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            self.get_logger().warning(
                "Invalid /pheromone_peak_reached message"
            )
            return

        best_target = None
        best_distance = float("inf")

        for target in self.targets:
            target_id = int(target["id"])

            if target_id in self.verified_targets:
                continue

            target_x = float(target["x"])
            target_y = float(target["y"])

            distance = math.hypot(
                husky_x - target_x,
                husky_y - target_y,
            )

            verification_radius = float(
                target["verification_radius"]
            )

            if (
                distance <= verification_radius
                and distance < best_distance
            ):
                best_target = target
                best_distance = distance

        if best_target is None:
            self.publish_event(
                "false_pheromone_peak",
                {
                    "husky_x": husky_x,
                    "husky_y": husky_y,
                    "peak_value": data.get("peak_value", 0.0),
                },
            )

            self.get_logger().warning(
                f"Peak at ({husky_x:.2f},{husky_y:.2f}) "
                "did not match any unverified target"
            )
            return

        target_id = int(best_target["id"])
        self.verified_targets.add(target_id)

        elapsed = time.time() - self.start_time

        verification = {
            "target_id": target_id,
            "target_name": best_target["name"],
            "target_type": best_target["type"],
            "verified_time_sec": elapsed,
            "husky_x": husky_x,
            "husky_y": husky_y,
            "distance_to_ground_truth": best_distance,
        }

        verified_message = String()
        verified_message.data = json.dumps(verification)
        self.verified_pub.publish(verified_message)

        clear = Point()
        clear.x = float(best_target["x"])
        clear.y = float(best_target["y"])
        clear.z = 2.5
        self.clear_pub.publish(clear)

        self.publish_event(
            "target_verified",
            {
                **verification,
                "verified_count": len(self.verified_targets),
                "total_targets": len(self.targets),
            },
        )

        self.get_logger().info(
            f"VERIFIED target {target_id}: "
            f"{best_target['name']} | "
            f"ground-truth error={best_distance:.2f}m"
        )


def main():
    rclpy.init()

    node = TargetVerifierResearch()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
