#!/usr/bin/env python3
"""
Experiment 3 — Scalability Test
================================
Tests communication saving as swarm size increases.
Simulates 3, 6, 9, 12 agents depositing pheromone
using the same event-trigger logic as drone_depositor.py

For each agent count:
- Runs for 60 seconds
- Measures communication saving
- Measures pheromone coverage
- Records role distribution
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import numpy as np
import math
import time
import random

class SimulatedAgent(Node):

    def __init__(self, agent_id, total_agents):
        super().__init__(f'sim_agent_{agent_id}')

        self.agent_id = agent_id
        self.total_agents = total_agents

        # Simulated position — spread agents across the map
        angle = (2 * math.pi * agent_id) / total_agents
        radius = 3.0
        self.x = radius * math.cos(angle)
        self.y = radius * math.sin(angle)
        self.vx = random.uniform(-0.5, 0.5)
        self.vy = random.uniform(-0.5, 0.5)

        # Last deposit position
        self.last_x = self.x
        self.last_y = self.y

        # Event trigger parameters
        self.min_move = 0.5
        self.low_threshold = 20

        # Pheromone map
        self.pheromone_grid = None
        self.map_info = None

        # Statistics
        self.deposits = 0
        self.skipped = 0

        # Publishers and subscribers
        self.deposit_pub = self.create_publisher(Point, '/pheromone_deposit', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/pheromone_map', self.map_callback, 10)

        # Position update at 50Hz (matching real drone rate)
        self.create_timer(0.02, self.update)

    def map_callback(self, msg):
        self.pheromone_grid = np.array(msg.data).reshape(
            msg.info.height, msg.info.width).astype(float)
        self.map_info = msg.info

    def get_pheromone_at(self, x, y):
        if self.pheromone_grid is None or self.map_info is None:
            return 0
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        if 0 <= row < self.pheromone_grid.shape[0] and \
           0 <= col < self.pheromone_grid.shape[1]:
            return float(self.pheromone_grid[row, col])
        return 0

    def update(self):
        # Move agent
        self.x += self.vx * 0.02
        self.y += self.vy * 0.02

        # Bounce off boundaries
        if abs(self.x) > 8:
            self.vx = -self.vx
        if abs(self.y) > 8:
            self.vy = -self.vy

        # Check triggers
        dx = self.x - self.last_x
        dy = self.y - self.last_y
        dist = math.sqrt(dx**2 + dy**2)

        local_conc = self.get_pheromone_at(self.x, self.y)

        if dist >= self.min_move or local_conc < self.low_threshold:
            msg = Point()
            msg.x = self.x
            msg.y = self.y
            msg.z = 0.8
            self.deposit_pub.publish(msg)
            self.last_x = self.x
            self.last_y = self.y
            self.deposits += 1
        else:
            self.skipped += 1

    def get_saving(self):
        total = self.deposits + self.skipped
        if total == 0:
            return 0
        return (self.skipped / total) * 100


class ScalabilityTest(Node):

    def __init__(self, n_agents, duration=60):
        super().__init__('scalability_test')
        self.n_agents = n_agents
        self.duration = duration
        self.start_time = time.time()
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/pheromone_map', self.map_callback, 10)
        self.pheromone_grid = None
        self.create_timer(1.0, self.check_done)
        self.get_logger().info(
            f'Scalability test started — {n_agents} agents, {duration}s duration')

    def map_callback(self, msg):
        self.pheromone_grid = np.array(msg.data).reshape(
            msg.info.height, msg.info.width).astype(float)

    def get_coverage(self):
        if self.pheromone_grid is None:
            return 0
        covered = np.sum(self.pheromone_grid > 5)
        total = self.pheromone_grid.size
        return (covered / total) * 100

    def check_done(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.get_logger().info(
                f'Test complete — {self.n_agents} agents')
            self.get_logger().info(
                f'Map coverage: {self.get_coverage():.1f}%')
            rclpy.shutdown()


def run_test(n_agents, duration=60):
    print(f'\n=== SCALABILITY TEST — {n_agents} AGENTS ===')
    rclpy.init()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    agents = []
    for i in range(n_agents):
        agent = SimulatedAgent(i, n_agents)
        agents.append(agent)
        executor.add_node(agent)

    monitor = ScalabilityTest(n_agents, duration)
    executor.add_node(monitor)

    try:
        executor.spin()
    except Exception:
        pass

    # Print results
    total_deposits = sum(a.deposits for a in agents)
    total_skipped = sum(a.skipped for a in agents)
    total = total_deposits + total_skipped
    overall_saving = (total_skipped / total * 100) if total > 0 else 0

    print(f'\n--- RESULTS: {n_agents} agents ---')
    print(f'Total deposits:  {total_deposits}')
    print(f'Total skipped:   {total_skipped}')
    print(f'Overall saving:  {overall_saving:.1f}%')
    print(f'Per agent avg:   {overall_saving:.1f}%')

    for node in agents:
        node.destroy_node()
    monitor.destroy_node()

    return {
        'agents': n_agents,
        'saving': overall_saving,
        'deposits': total_deposits,
        'skipped': total_skipped
    }


if __name__ == '__main__':
    results = []

    for n in [3, 6, 9, 12]:
        result = run_test(n, duration=60)
        results.append(result)
        time.sleep(2)

    print('\n\n=== EXPERIMENT 3 — SCALABILITY SUMMARY ===')
    print(f'{"Agents":<10} {"Saving %":<15} {"Deposits":<15} {"Skipped":<15}')
    print('-' * 55)
    for r in results:
        print(f'{r["agents"]:<10} {r["saving"]:<15.1f} {r["deposits"]:<15} {r["skipped"]:<15}')
