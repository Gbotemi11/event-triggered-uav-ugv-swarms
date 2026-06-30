#!/usr/bin/env python3
"""
Experiment 4 — Robustness Test (Fixed)
========================================
Tests swarm behaviour when one agent fails mid-mission.

Procedure:
1. Run 3 agents for 30 seconds (baseline)
2. Kill agent 1 at t=30s
3. Run remaining 2 agents for another 30 seconds
4. Measure pheromone map degradation and recovery
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import numpy as np
import math
import time
import random
import threading

class RobustnessAgent(Node):

    def __init__(self, agent_id, total_agents):
        super().__init__(f'robust_agent_{agent_id}')

        self.agent_id = agent_id
        self.active = True
        self.phase = 1  # 1 = before failure, 2 = after failure

        # Spread agents across map
        angle = (2 * math.pi * agent_id) / total_agents
        radius = 3.0
        self.x = radius * math.cos(angle)
        self.y = radius * math.sin(angle)
        self.vx = random.uniform(-0.3, 0.3) + 0.2
        self.vy = random.uniform(-0.3, 0.3) + 0.2

        self.last_x = self.x
        self.last_y = self.y

        # Event trigger parameters
        self.min_move = 0.5
        self.low_threshold = 20

        # Pheromone map
        self.pheromone_grid = None
        self.map_info = None

        # Statistics split by phase
        self.deposits = {1: 0, 2: 0}
        self.skipped = {1: 0, 2: 0}

        self.deposit_pub = self.create_publisher(
            Point, '/pheromone_deposit', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/pheromone_map', self.map_callback, 10)

        self.create_timer(0.02, self.update)

    def map_callback(self, msg):
        self.pheromone_grid = np.array(msg.data).reshape(
            msg.info.height, msg.info.width).astype(float)
        self.map_info = msg.info

    def get_pheromone_at(self, x, y):
        if self.pheromone_grid is None or self.map_info is None:
            return 0
        col = int((x - self.map_info.origin.position.x) /
                  self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) /
                  self.map_info.resolution)
        if 0 <= row < self.pheromone_grid.shape[0] and \
           0 <= col < self.pheromone_grid.shape[1]:
            return float(self.pheromone_grid[row, col])
        return 0

    def fail(self):
        self.active = False
        self.get_logger().info(
            f'Agent {self.agent_id} FAILED — simulating dropout')

    def set_phase(self, phase):
        self.phase = phase

    def update(self):
        if not self.active:
            return

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
            self.deposits[self.phase] += 1
        else:
            self.skipped[self.phase] += 1

    def get_saving(self, phase):
        total = self.deposits[phase] + self.skipped[phase]
        if total == 0:
            return 0
        return (self.skipped[phase] / total) * 100


class RobustnessMonitor(Node):

    def __init__(self):
        super().__init__('robustness_monitor')
        self.pheromone_grid = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/pheromone_map', self.map_callback, 10)
        self.coverage_history = []

    def map_callback(self, msg):
        self.pheromone_grid = np.array(msg.data).reshape(
            msg.info.height, msg.info.width).astype(float)

    def get_coverage(self):
        if self.pheromone_grid is None:
            return 0
        covered = np.sum(self.pheromone_grid > 5)
        return (covered / self.pheromone_grid.size) * 100

    def get_avg_concentration(self):
        if self.pheromone_grid is None:
            return 0
        return float(np.mean(self.pheromone_grid))


def run_agent(agent):
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(agent)
    try:
        executor.spin()
    except Exception:
        pass
    finally:
        agent.destroy_node()


def main():
    print('\n=== EXPERIMENT 4 — ROBUSTNESS TEST ===')
    rclpy.init()

    agents = [RobustnessAgent(i, 3) for i in range(3)]
    monitor = RobustnessMonitor()

    # Run each agent in its own thread
    agent_threads = []
    for agent in agents:
        t = threading.Thread(target=run_agent, args=(agent,), daemon=True)
        agent_threads.append(t)
        t.start()

    # Run monitor in its own thread
    from rclpy.executors import SingleThreadedExecutor
    monitor_executor = SingleThreadedExecutor()
    monitor_executor.add_node(monitor)
    monitor_thread = threading.Thread(
        target=monitor_executor.spin, daemon=True)
    monitor_thread.start()

    start_time = time.time()

    # Phase 1 — all 3 agents running (30 seconds)
    print('\nPhase 1 — all 3 agents running...')
    while time.time() - start_time < 30:
        time.sleep(5)
        elapsed = time.time() - start_time
        coverage = monitor.get_coverage()
        avg_conc = monitor.get_avg_concentration()
        print(f't={elapsed:.0f}s — coverage: {coverage:.1f}% '
              f'avg_conc: {avg_conc:.2f} — all 3 agents active')
        monitor.coverage_history.append(
            (elapsed, coverage, avg_conc, 'BEFORE'))

    # Phase 2 — kill agent 1
    print('\n>>> AGENT 1 FAILURE AT t=30s <<<')
    agents[1].fail()
    for agent in agents:
        agent.set_phase(2)
    failure_time = time.time()

    # Phase 3 — remaining 2 agents continue (30 seconds)
    print('\nPhase 2 — 2 agents remaining...')
    while time.time() - failure_time < 30:
        time.sleep(5)
        elapsed_total = time.time() - start_time
        elapsed_after = time.time() - failure_time
        coverage = monitor.get_coverage()
        avg_conc = monitor.get_avg_concentration()
        print(f't={elapsed_total:.0f}s (+{elapsed_after:.0f}s after failure) — '
              f'coverage: {coverage:.1f}% avg_conc: {avg_conc:.2f}')
        monitor.coverage_history.append(
            (elapsed_total, coverage, avg_conc, 'AFTER'))

    # Print results
    print('\n\n=== EXPERIMENT 4 RESULTS ===')

    print('\nPhase 1 — all 3 agents (0-30s):')
    for agent in agents:
        saving = agent.get_saving(1)
        print(f'  Agent {agent.agent_id}: '
              f'deposits={agent.deposits[1]} '
              f'skipped={agent.skipped[1]} '
              f'saving={saving:.1f}%')

    print('\nPhase 2 — after agent 1 failure (30-60s):')
    for agent in agents:
        if not agent.active and agent.deposits[2] == 0:
            print(f'  Agent {agent.agent_id}: FAILED — no activity')
            continue
        saving = agent.get_saving(2)
        print(f'  Agent {agent.agent_id}: '
              f'deposits={agent.deposits[2]} '
              f'skipped={agent.skipped[2]} '
              f'saving={saving:.1f}%')

    print('\nCoverage history:')
    print(f'  {"Time":<8} {"Phase":<8} {"Coverage":<12} {"Avg Conc":<10}')
    print('  ' + '-' * 40)
    for ts, cov, avg, phase in monitor.coverage_history:
        print(f'  t={ts:<6.0f} {phase:<8} {cov:<12.1f} {avg:<10.2f}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
