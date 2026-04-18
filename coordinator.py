# coordinator.py
import networkx as nx
from typing import Dict, List, Tuple, Optional
from models import Robot, LaneType

class Coordinator:
    def __init__(self, G: nx.DiGraph):
        self.G = G
        self.reservation_table: Dict[Tuple[int, int], List[Tuple[float, float, int]]] = {}

    def calculate_speed(self, lane) -> float:
        """Lane-based speed control (smooth, readable factors).
        Narrow/human zones are strictly limited. Safety overrides speed."""
        base = lane.max_speed
        cong_factor = max(0.4, 1.0 - 0.6 * lane.congestion_score)
        safety_factor = lane.safety_level / 5.0
        type_factor = 1.0
        if lane.lane_type == LaneType.NARROW:
            type_factor = 0.5
        elif lane.lane_type == LaneType.HUMAN_ZONE:
            type_factor = 0.3
        elif lane.lane_type == LaneType.INTERSECTION:
            type_factor = 0.8
        return base * cong_factor * safety_factor * type_factor

    def plan_path(self, robot: Robot, current_time: float = 0.0) -> Optional[List[int]]:
        """Replanning entry point (called when blocked/congested)."""
        from planner import plan_path
        return plan_path(self.G, robot.current_node, robot.goal_node)

    def can_reserve(self, lane_key: Tuple[int, int], start_t: float, end_t: float, robot_id: int) -> bool:
        if lane_key not in self.reservation_table:
            return True
        for s, e, rid in self.reservation_table[lane_key]:
            if rid == robot_id:
                continue
            if max(start_t, s) < min(end_t, e):
                return False
        return True

    def add_reservation(self, lane_key: Tuple[int, int], start_t: float, end_t: float, robot_id: int):
        if lane_key not in self.reservation_table:
            self.reservation_table[lane_key] = []
        self.reservation_table[lane_key].append((start_t, end_t, robot_id))

    def clean_reservations(self, current_time: float):
        for key in list(self.reservation_table.keys()):
            self.reservation_table[key] = [res for res in self.reservation_table[key] if res[1] > current_time]
            if not self.reservation_table[key]:
                del self.reservation_table[key]

    def detect_and_resolve_deadlock(self, waiting: Dict[int, set], robots: Dict[int, Robot]) -> int:
        """Deadlock handling (as required):
        1. Build wait-for graph (A waits for B if waiting on B's reserved lane).
        2. Detect cycle with NetworkX.
        3. Resolve: lowest-priority robot (highest ID) forced to replan.
        Simple, deterministic, and safe."""
        if not waiting:
            return 0
        temp_g = nx.DiGraph()
        for r_id, waits in waiting.items():
            for w_id in waits:
                temp_g.add_edge(r_id, w_id)
        try:
            cycle = nx.find_cycle(temp_g)
            cycle_nodes = set()
            for a, b in cycle:
                cycle_nodes.add(a)
                cycle_nodes.add(b)
            if cycle_nodes:
                to_reroute_id = max(cycle_nodes)  # highest ID = lowest priority
                if to_reroute_id in robots:
                    robots[to_reroute_id].current_path = []
                    return 1
        except nx.NetworkXNoCycle:
            pass
        return 0