#!/usr/bin/env python3
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)

from scenario_utils import load_scenario, get_sector_for_uav


class DroneSectorPatrol(Node):
    def __init__(self, uav_config, scenario):
        drone_id = int(uav_config["id"])
        super().__init__(f"sector_patrol_drone_{drone_id}")

        self.scenario = scenario
        self.uav = uav_config
        self.drone_id = drone_id
        self.namespace = uav_config["namespace"]
        self.system_id = int(uav_config["system_id"])
        self.target_altitude = float(uav_config["altitude"])
        self.spawn_x = float(uav_config["spawn_x"])
        self.spawn_y = float(uav_config["spawn_y"])

        sector = get_sector_for_uav(scenario, self.drone_id)
        self.waypoints_world = sector["waypoints"]
        self.current_wp = 0

        self.counter = 0
        self.armed = False
        self.offboard_set = False
        self.start_time = time.time()
        self.has_position = False
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0

        self.failed = False
        self.failure_announced = False
        self.failed_uav_id = int(os.environ.get(
            "FAILED_UAV_ID",
            scenario["experiments"]["failure_test"]["failed_uav_id"],
        ))
        self.failure_time = float(os.environ.get(
            "FAILURE_TIME_SEC",
            scenario["experiments"]["failure_test"]["failure_time_sec"],
        ))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        prefix = self.namespace

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f"{prefix}/fmu/in/offboard_control_mode",
            qos,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f"{prefix}/fmu/in/trajectory_setpoint",
            qos,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            f"{prefix}/fmu/in/vehicle_command",
            qos,
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f"{prefix}/fmu/out/vehicle_local_position_v1",
            self.local_position_callback,
            qos,
        )

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Drone {self.drone_id} sector patrol ready | "
            f"spawn=({self.spawn_x},{self.spawn_y}) | "
            f"sector={self.uav['sector']} | "
            f"waypoints={len(self.waypoints_world)}"
        )

    def local_position_callback(self, msg):
        self.local_x = float(msg.x)
        self.local_y = float(msg.y)
        self.local_z = float(msg.z)
        self.has_position = True

    def world_to_local_setpoint(self, wx, wy):
        return wx - self.spawn_x, wy - self.spawn_y

    def current_target_local(self):
        wx, wy = self.waypoints_world[self.current_wp]
        return self.world_to_local_setpoint(float(wx), float(wy))

    def control_loop(self):
        elapsed = time.time() - self.start_time

        if self.drone_id == self.failed_uav_id and elapsed >= self.failure_time:
            self.failed = True
            if not self.failure_announced:
                self.get_logger().warn(
                    f"Drone {self.drone_id} simulated failure at {elapsed:.1f}s. "
                    f"Holding position and no longer advancing sector patrol."
                )
                self.failure_announced = True

        self.publish_offboard_mode()
        self.publish_trajectory()

        if self.counter == 150 and not self.offboard_set:
            self.set_offboard_mode()
            self.offboard_set = True

        if self.counter == 160 and not self.armed:
            self.arm()
            self.armed = True

        if self.counter > 155 and self.counter % 50 == 0:
            self.set_offboard_mode()

        self.counter += 1

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_trajectory(self):
        msg = TrajectorySetpoint()
        elapsed = time.time() - self.start_time

        if elapsed < 10.0:
            x = 0.0
            y = 0.0
        elif self.failed:
            x = self.local_x
            y = self.local_y
        else:
            x, y = self.current_target_local()

            if self.has_position:
                dist = math.hypot(x - self.local_x, y - self.local_y)
                if dist < 1.5:
                    self.current_wp = (self.current_wp + 1) % len(self.waypoints_world)
                    wx, wy = self.waypoints_world[self.current_wp]
                    self.get_logger().info(
                        f"Drone {self.drone_id} switching to waypoint "
                        f"{self.current_wp}: world=({wx:.1f},{wy:.1f})"
                    )
                    x, y = self.current_target_local()

        msg.position = [float(x), float(y), self.target_altitude]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = self.system_id
        msg.target_component = 1
        msg.source_system = self.system_id
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)
        self.get_logger().info(f"Drone {self.drone_id} ARM sent.")

    def set_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.target_system = self.system_id
        msg.target_component = 1
        msg.source_system = self.system_id
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


def run_drone(uav_config, scenario):
    node = DroneSectorPatrol(uav_config, scenario)
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

    print("Research sector patrol running. Press Ctrl+C to stop.")
    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        print("Shutting down sector patrol.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
