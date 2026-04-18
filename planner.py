# planner.py
import networkx as nx
from models import LaneType
from typing import Optional, List

def calculate_edge_cost(G: nx.DiGraph, u: int, v: int) -> float:
    """Simple readable cost: distance + congestion + safety + type penalties.
    Safety is a hard penalty (higher cost for unsafe lanes)."""
    lane = G.edges[u, v]['lane']
    length = G.edges[u, v]['length']
    base = length
    cong_penalty = length * lane.congestion_score * 3.5
    safety_penalty = length * (5 - lane.safety_level) * 1.5
    type_penalty = 0.0
    if lane.lane_type == LaneType.NARROW:
        type_penalty = length * 3.0
    elif lane.lane_type == LaneType.HUMAN_ZONE:
        type_penalty = length * 8.0
    elif lane.lane_type == LaneType.INTERSECTION:
        type_penalty = length * 1.5
    return base + cong_penalty + safety_penalty + type_penalty

def plan_path(G: nx.DiGraph, start: int, goal: int) -> Optional[List[int]]:
    """A* planning using NetworkX (weight is callable - verified in docs)."""
    def heuristic(n1, n2):
        p1 = G.nodes[n1]['pos']
        p2 = G.nodes[n2]['pos']
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    def weight(u, v, d):
        return calculate_edge_cost(G, u, v)

    try:
        return nx.astar_path(G, start, goal, heuristic=heuristic, weight=weight)
    except nx.NetworkXNoPath:
        return None