# simulator.py
import networkx as nx
from typing import List, Dict, Tuple
from models import Robot, SimulationMetrics
from coordinator import Coordinator
from planner import plan_path
import os
import json
from datetime import datetime

class Simulator:
    def __init__(self, G: nx.DiGraph, robots_list: List[Robot], coordinator: Coordinator, test_name: str = "default"):
        self.G = G
        self.robots: Dict[int, Robot] = {r.robot_id: r for r in robots_list}
        self.coordinator = coordinator
        self.current_time = 0.0
        self.dt = 1.0
        self.max_time = 300.0
        self.metrics = SimulationMetrics()
        self.metrics.total_robots = len(robots_list)
        self.trajectories: Dict[int, List[Tuple[float, int]]] = {rid: [] for rid in self.robots}
        self.num_replans = 0
        self.test_name = test_name

    def get_waiting_robots(self) -> Dict[int, set]:
        waiting: Dict[int, set] = {}
        for rid, robot in self.robots.items():
            if robot.is_moving or robot.current_node == robot.goal_node or not robot.current_path:
                continue
            next_node = robot.current_path[0]
            lane_key = (robot.current_node, next_node)
            if lane_key not in self.coordinator.reservation_table:
                continue
            for s, e, holder in self.coordinator.reservation_table[lane_key]:
                if holder != rid and max(s, self.current_time) < min(e, self.current_time + 100):
                    if rid not in waiting:
                        waiting[rid] = set()
                    waiting[rid].add(holder)
                    break
        return waiting

    def update_congestion(self):
        for u, v in list(self.G.edges()):
            lane = self.G.edges[u, v]['lane']
            active = 0
            lane_key = (u, v)
            if lane_key in self.coordinator.reservation_table:
                for s, e, _ in self.coordinator.reservation_table[lane_key]:
                    if s <= self.current_time <= e:
                        active += 1
            lane.congestion_score = min(1.0, (lane.historical_usage / 20.0) + (active * 0.4))

    def save_test_results(self, test_name: str):
        """Save all results and plots per test (as required).
        Creates: test_results/{test_name}/metrics.json, trajectories.json, heatmap.png"""
        base_dir = "test_results"
        os.makedirs(base_dir, exist_ok=True)
        test_dir = os.path.join(base_dir, test_name)
        os.makedirs(test_dir, exist_ok=True)

        # Metrics
        metrics_dict = {
            "test_name": test_name,
            "timestamp": datetime.now().isoformat(),
            "total_robots": self.metrics.total_robots,
            "completed_robots": self.metrics.completed_robots,
            "success_rate": round(self.metrics.success_rate, 2),
            "makespan": round(self.metrics.makespan, 2),
            "avg_travel_time": round(self.metrics.avg_travel_time, 2),
            "avg_waiting_time": round(self.metrics.avg_waiting_time, 2),
            "throughput": round(self.metrics.throughput, 3),
            "num_replans": self.metrics.num_replans,
            "num_deadlocks_detected": self.metrics.num_deadlocks_detected,
            "num_deadlocks_resolved": self.metrics.num_deadlocks_resolved,
            "collisions_prevented": self.metrics.collisions_prevented,
            "lane_utilization": self.metrics.lane_utilization
        }
        with open(os.path.join(test_dir, "metrics.json"), "w") as f:
            json.dump(metrics_dict, f, indent=2)

        # Trajectories
        traj_dict = {rid: [(float(t), node) for t, node in traj] for rid, traj in self.trajectories.items()}
        with open(os.path.join(test_dir, "trajectories.json"), "w") as f:
            json.dump(traj_dict, f, indent=2)

        # Heatmap 
        from visualisation import visualize_lane_heatmap
        heatmap_path = os.path.join(test_dir, "lane_heatmap.png")
        visualize_lane_heatmap(self.G, save_path=heatmap_path)

        print(f"Test '{test_name}' results saved to ./{test_dir}/")

    def run(self):
        # Initial paths
        for robot in self.robots.values():
            path = plan_path(self.G, robot.current_node, robot.goal_node)
            if path and len(path) > 1:
                robot.current_path = path[1:]
                self.trajectories[robot.robot_id].append((self.current_time, robot.current_node))

        while self.current_time < self.max_time:
            if all(r.current_node == r.goal_node for r in self.robots.values()):
                break

            self.coordinator.clean_reservations(self.current_time)
            self.update_congestion()

            # Deadlock check
            waiting = self.get_waiting_robots()
            resolved = self.coordinator.detect_and_resolve_deadlock(waiting, self.robots)
            if resolved > 0:
                self.metrics.num_deadlocks_detected += 1
                self.metrics.num_deadlocks_resolved += resolved

            # Process robots by priority
            sorted_robots = sorted(self.robots.values(), key=lambda r: r.priority)
            for robot in sorted_robots:
                if robot.current_node == robot.goal_node:
                    continue

                # Arrival handling
                if robot.is_moving and robot.arrival_time is not None and self.current_time >= robot.arrival_time:
                    robot.current_node = robot.target_node
                    robot.is_moving = False
                    robot.target_node = None
                    robot.arrival_time = None
                    if robot.current_lane:
                        u, v = robot.current_lane
                        lane = self.G.edges[u, v]['lane']
                        lane.historical_usage += 1
                        robot.current_lane = None
                    self.trajectories[robot.robot_id].append((self.current_time, robot.current_node))
                    if robot.current_path:
                        robot.current_path = robot.current_path[1:]
                    continue

                # Try to move or replan
                if not robot.is_moving:
                    replan_needed = len(robot.current_path) == 0 or \
                                    (robot.current_path and self.G.edges[robot.current_node, robot.current_path[0]]['lane'].congestion_score > 0.5)
                    if replan_needed:
                        path = plan_path(self.G, robot.current_node, robot.goal_node)
                        if path and len(path) > 1:
                            robot.current_path = path[1:]
                            self.num_replans += 1

                    if robot.current_path:
                        next_node = robot.current_path[0]
                        lane_key = (robot.current_node, next_node)
                        if lane_key not in self.G.edges:
                            robot.current_path = []
                            continue
                        lane = self.G.edges[lane_key]['lane']
                        speed = self.coordinator.calculate_speed(lane)
                        length = self.G.edges[lane_key]['length']
                        trav_time = length / max(speed, 0.1)
                        start_t = self.current_time
                        end_t = start_t + trav_time
                        if self.coordinator.can_reserve(lane_key, start_t, end_t, robot.robot_id):
                            self.coordinator.add_reservation(lane_key, start_t, end_t, robot.robot_id)
                            robot.is_moving = True
                            robot.target_node = next_node
                            robot.arrival_time = end_t
                            robot.current_lane = lane_key
                            self.trajectories[robot.robot_id].append((self.current_time, robot.current_node))
                        else:
                            self.metrics.collisions_prevented += 1
                            # Dynamic replan on denial
                            path = plan_path(self.G, robot.current_node, robot.goal_node)
                            if path and len(path) > 1:
                                robot.current_path = path[1:]
                                self.num_replans += 1
                    # Count waiting time
                    if not robot.is_moving and robot.current_node != robot.goal_node:
                        robot.waiting_time += self.dt

            self.current_time += self.dt

        self.finalize_metrics()
        from visualisation import visualize_lane_heatmap
        visualize_lane_heatmap(self.G)

        if hasattr(self, 'test_name'):
            self.save_test_results(self.test_name)

    def finalize_metrics(self):
        completed = [r for r in self.robots.values() if r.current_node == r.goal_node]
        self.metrics.completed_robots = len(completed)
        self.metrics.success_rate = (self.metrics.completed_robots / self.metrics.total_robots * 100) if self.metrics.total_robots > 0 else 0.0
        travel_times = []
        for r in completed:
            if self.trajectories[r.robot_id]:
                arrival = self.trajectories[r.robot_id][-1][0]
                travel = arrival - r.start_time
                travel_times.append(travel)
        if travel_times:
            self.metrics.total_travel_time = sum(travel_times)
            self.metrics.avg_travel_time = sum(travel_times) / len(travel_times)
            self.metrics.avg_waiting_time = sum(r.waiting_time for r in completed) / len(completed)
            self.metrics.makespan = max(self.trajectories[r.robot_id][-1][0] for r in completed)
        self.metrics.throughput = self.metrics.completed_robots / (self.current_time + 1e-6)
        self.metrics.num_replans = self.num_replans
        for u, v in self.G.edges():
            lane = self.G.edges[u, v]['lane']
            self.metrics.lane_utilization[lane.lane_id] = lane.historical_usage