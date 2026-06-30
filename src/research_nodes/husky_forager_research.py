#!/usr/bin/env python3

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String

from scenario_utils import load_scenario


class HuskyForagerResearch(Node):
    """
    Pheromone-driven UGV controller.

    Important:
    - This controller does not read target coordinates.
    - Navigation is derived entirely from /pheromone_map.
    - Ground-truth target verification is handled by a separate node.
    """

    def __init__(self):
        super().__init__("husky_forager_research")

        scenario = load_scenario()
        husky_cfg = scenario["robots"]["husky"]

        self.spawn_x = float(husky_cfg["spawn_x"])
        self.spawn_y = float(husky_cfg["spawn_y"])

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.world_x = self.spawn_x
        self.world_y = self.spawn_y
        self.yaw = 0.0
        self.have_odom = False

        self.pheromone_grid = None
        self.map_info = None

        # Motion parameters
        self.linear_speed = 0.40
        self.angular_gain = 0.9
        self.max_angular_speed = 0.6

        # Pheromone-following parameters
        self.minimum_signal = 2.0
        self.local_follow_radius_m = 5.0
        self.global_signal_threshold = 8.0
        self.global_distance_penalty = 0.10

        # Peak-detection parameters
        self.target_peak_threshold = 45.0
        self.peak_search_radius_m = 1.5
        self.peak_reach_radius_m = 0.8
        self.peak_hold_sec = 1.5
        self.rearm_threshold = 5.0

        self.peak_candidate_since = None
        self.awaiting_clear = False

        # Stabilise pheromone guidance and prevent immediate revisits.
        self.guidance_x = None
        self.guidance_y = None
        self.guidance_signal = 0.0
        self.guidance_last_update = 0.0
        self.guidance_hold_sec = 2.0
        self.guidance_alpha = 0.20
        self.guidance_reach_radius_m = 0.40

        self.post_peak_cooldown_sec = 15.0
        self.cooldown_until = 0.0

        self.start_time = time.time()
        self.last_status_time = 0.0

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/pheromone_map",
            self.map_callback,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/husky_0/odom",
            self.odom_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/husky_0/cmd_vel",
            10,
        )

        self.peak_pub = self.create_publisher(
            String,
            "/pheromone_peak_reached",
            10,
        )

        self.event_pub = self.create_publisher(
            String,
            "/swarm/event",
            10,
        )

        self.create_timer(0.1, self.navigate)

        self.get_logger().info(
            "Pheromone-field Husky controller started. "
            "No target coordinates are loaded by the controller."
        )

    def publish_event(self, event_type, payload):
        message = String()
        data = dict(payload)
        data["event_type"] = event_type
        data["time_sec"] = time.time() - self.start_time
        message.data = json.dumps(data)
        self.event_pub.publish(message)

    def odom_callback(self, msg):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)

        self.world_x = self.spawn_x + self.odom_x
        self.world_y = self.spawn_y + self.odom_y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        self.have_odom = True

    def map_callback(self, msg):
        try:
            grid = np.asarray(msg.data, dtype=np.float32)
            self.pheromone_grid = grid.reshape(
                msg.info.height,
                msg.info.width,
            )
            self.map_info = msg.info
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"Invalid pheromone map: {exc}")

    def world_to_grid(self, x, y):
        if self.map_info is None:
            return None

        col = int(
            (x - self.map_info.origin.position.x)
            / self.map_info.resolution
        )
        row = int(
            (y - self.map_info.origin.position.y)
            / self.map_info.resolution
        )

        if (
            self.pheromone_grid is not None
            and 0 <= row < self.pheromone_grid.shape[0]
            and 0 <= col < self.pheromone_grid.shape[1]
        ):
            return row, col

        return None

    def grid_to_world(self, row, col):
        resolution = self.map_info.resolution

        x = (
            self.map_info.origin.position.x
            + (col + 0.5) * resolution
        )
        y = (
            self.map_info.origin.position.y
            + (row + 0.5) * resolution
        )

        return x, y

    def local_concentration(self):
        cell = self.world_to_grid(self.world_x, self.world_y)

        if cell is None:
            return 0.0

        row, col = cell
        return float(self.pheromone_grid[row, col])

    def strongest_local_peak(self):
        """
        Return the strongest pheromone cell close to the UGV.
        Used only to determine whether the UGV has reached a field peak.
        """
        cell = self.world_to_grid(self.world_x, self.world_y)

        if cell is None:
            return None

        row, col = cell
        radius_cells = max(
            1,
            int(
                self.peak_search_radius_m
                / self.map_info.resolution
            ),
        )

        r0 = max(0, row - radius_cells)
        r1 = min(
            self.pheromone_grid.shape[0],
            row + radius_cells + 1,
        )
        c0 = max(0, col - radius_cells)
        c1 = min(
            self.pheromone_grid.shape[1],
            col + radius_cells + 1,
        )

        window = self.pheromone_grid[r0:r1, c0:c1]

        if window.size == 0:
            return None

        local_index = np.unravel_index(
            int(np.argmax(window)),
            window.shape,
        )

        peak_row = r0 + local_index[0]
        peak_col = c0 + local_index[1]
        peak_value = float(
            self.pheromone_grid[peak_row, peak_col]
        )

        peak_x, peak_y = self.grid_to_world(
            peak_row,
            peak_col,
        )

        distance = math.hypot(
            peak_x - self.world_x,
            peak_y - self.world_y,
        )

        return peak_x, peak_y, peak_value, distance

    def local_guidance_point(self):
        """
        Compute a pheromone-weighted centroid inside a local window.

        The UGV moves toward locally stronger pheromone rather than toward
        a target coordinate stored in the scenario.
        """
        cell = self.world_to_grid(self.world_x, self.world_y)

        if cell is None:
            return None

        row, col = cell
        radius_cells = max(
            1,
            int(
                self.local_follow_radius_m
                / self.map_info.resolution
            ),
        )

        r0 = max(0, row - radius_cells)
        r1 = min(
            self.pheromone_grid.shape[0],
            row + radius_cells + 1,
        )
        c0 = max(0, col - radius_cells)
        c1 = min(
            self.pheromone_grid.shape[1],
            col + radius_cells + 1,
        )

        window = self.pheromone_grid[r0:r1, c0:c1]

        if window.size == 0:
            return None

        local_max = float(np.max(window))

        if local_max < self.minimum_signal:
            return None

        # Ignore weak background values and emphasise strong trail segments.
        cutoff = max(
            self.minimum_signal,
            0.35 * local_max,
        )

        rows, cols = np.where(window >= cutoff)

        if rows.size == 0:
            return None

        values = window[rows, cols].astype(np.float64)

        # Squaring gives stronger pheromone cells more influence.
        weights = values * values

        global_rows = rows + r0
        global_cols = cols + c0

        world_x = (
            self.map_info.origin.position.x
            + (global_cols + 0.5)
            * self.map_info.resolution
        )
        world_y = (
            self.map_info.origin.position.y
            + (global_rows + 0.5)
            * self.map_info.resolution
        )

        centroid_x = float(
            np.sum(weights * world_x)
            / np.sum(weights)
        )
        centroid_y = float(
            np.sum(weights * world_y)
            / np.sum(weights)
        )

        return centroid_x, centroid_y, local_max

    def global_acquisition_point(self):
        """
        Find a significant pheromone region when no trail is locally visible.

        This uses only the shared pheromone field. It does not access target
        coordinates or target metadata.
        """
        if self.pheromone_grid is None:
            return None

        rows, cols = np.where(
            self.pheromone_grid >= self.global_signal_threshold
        )

        if rows.size == 0:
            return None

        values = self.pheromone_grid[rows, cols].astype(
            np.float64
        )

        world_x = (
            self.map_info.origin.position.x
            + (cols + 0.5)
            * self.map_info.resolution
        )
        world_y = (
            self.map_info.origin.position.y
            + (rows + 0.5)
            * self.map_info.resolution
        )

        distances = np.hypot(
            world_x - self.world_x,
            world_y - self.world_y,
        )

        scores = (
            values
            - self.global_distance_penalty * distances
        )

        best = int(np.argmax(scores))

        return (
            float(world_x[best]),
            float(world_y[best]),
            float(values[best]),
        )

    def publish_peak_reached(self, peak):
        peak_x, peak_y, peak_value, distance = peak

        data = {
            "husky_x": self.world_x,
            "husky_y": self.world_y,
            "peak_x": peak_x,
            "peak_y": peak_y,
            "peak_value": peak_value,
            "distance_to_peak": distance,
            "time_sec": time.time() - self.start_time,
        }

        message = String()
        message.data = json.dumps(data)
        self.peak_pub.publish(message)

        self.publish_event(
            "pheromone_peak_reached",
            data,
        )

        self.get_logger().info(
            f"Pheromone peak reached at "
            f"({peak_x:.2f}, {peak_y:.2f}), "
            f"value={peak_value:.1f}"
        )

    def stop(self):
        self.cmd_pub.publish(Twist())

    def check_peak_arrival(self):
        peak = self.strongest_local_peak()

        if peak is None:
            self.peak_candidate_since = None
            return False

        _, _, peak_value, distance = peak

        valid_peak = (
            peak_value >= self.target_peak_threshold
            and distance <= self.peak_reach_radius_m
        )

        if not valid_peak:
            self.peak_candidate_since = None
            return False

        now = time.time()

        if self.peak_candidate_since is None:
            self.peak_candidate_since = now
            return False

        if now - self.peak_candidate_since < self.peak_hold_sec:
            return False

        self.stop()
        self.publish_peak_reached(peak)

        self.awaiting_clear = True
        self.peak_candidate_since = None

        self.cooldown_until = (
            time.time() + self.post_peak_cooldown_sec
        )

        self.guidance_x = None
        self.guidance_y = None
        self.guidance_signal = 0.0

        return True

    def drive_toward(self, target_x, target_y, mode, signal):
        dx = target_x - self.world_x
        dy = target_y - self.world_y

        target_angle = math.atan2(dy, dx)

        angle_error = target_angle - self.yaw

        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi

        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        cmd = Twist()

        cmd.angular.z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                self.angular_gain * angle_error,
            ),
        )

        # Turn first when the guidance direction is far from the heading.
        if abs(angle_error) < 0.65:
            heading_scale = max(
                0.25,
                1.0 - abs(angle_error) / 0.65,
            )
            cmd.linear.x = (
                self.linear_speed * heading_scale
            )

        self.cmd_pub.publish(cmd)

        now = time.time()

        if now - self.last_status_time >= 2.0:
            self.last_status_time = now
            self.get_logger().info(
                f"mode={mode} | "
                f"position=({self.world_x:.2f},{self.world_y:.2f}) | "
                f"guidance=({target_x:.2f},{target_y:.2f}) | "
                f"signal={signal:.1f}"
            )

    def navigate(self):
        if (
            not self.have_odom
            or self.pheromone_grid is None
            or self.map_info is None
        ):
            self.stop()
            return

        concentration = self.local_concentration()

        if (
            not self.awaiting_clear
            and time.time() < self.cooldown_until
        ):
            self.stop()
            return

        if self.awaiting_clear:
            self.stop()

            # Rearm after the verifier clears the reached field peak.
            if concentration <= self.rearm_threshold:
                self.awaiting_clear = False
                self.publish_event(
                    "pheromone_follower_rearmed",
                    {
                        "husky_x": self.world_x,
                        "husky_y": self.world_y,
                    },
                )

            return

        if self.check_peak_arrival():
            return

        local_goal = self.local_guidance_point()

        if local_goal is not None:
            raw_x, raw_y, raw_signal = local_goal
            now = time.time()

            guidance_reached = (
                self.guidance_x is not None
                and math.hypot(
                    self.guidance_x - self.world_x,
                    self.guidance_y - self.world_y,
                ) <= self.guidance_reach_radius_m
            )

            update_guidance = (
                self.guidance_x is None
                or now - self.guidance_last_update
                >= self.guidance_hold_sec
                or guidance_reached
            )

            if update_guidance:
                if self.guidance_x is None:
                    self.guidance_x = raw_x
                    self.guidance_y = raw_y
                else:
                    alpha = self.guidance_alpha

                    self.guidance_x = (
                        (1.0 - alpha) * self.guidance_x
                        + alpha * raw_x
                    )
                    self.guidance_y = (
                        (1.0 - alpha) * self.guidance_y
                        + alpha * raw_y
                    )

                self.guidance_signal = raw_signal
                self.guidance_last_update = now

            self.drive_toward(
                self.guidance_x,
                self.guidance_y,
                "local_following",
                self.guidance_signal,
            )
            return

        self.guidance_x = None
        self.guidance_y = None
        self.guidance_signal = 0.0

        acquisition_goal = self.global_acquisition_point()

        if acquisition_goal is not None:
            goal_x, goal_y, signal = acquisition_goal
            self.drive_toward(
                goal_x,
                goal_y,
                "field_acquisition",
                signal,
            )
            return

        self.stop()


def main():
    rclpy.init()

    node = HuskyForagerResearch()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
