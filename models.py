# models.py
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict
import enum

class LaneType(str, enum.Enum):
    NORMAL = "normal"
    NARROW = "narrow"
    INTERSECTION = "intersection"
    HUMAN_ZONE = "human_zone"

@dataclass
class Lane:
    lane_id: str
    max_speed: float
    safety_level: int  # 1-5 (5 = safest)
    lane_type: LaneType
    congestion_score: float = 0.0
    historical_usage: int = 0
    length: float = 10.0

@dataclass
class Robot:
    robot_id: int
    start_node: int
    goal_node: int
    current_node: int
    priority: int  # lower number = higher priority (processed first)
    current_path: List[int] = field(default_factory=list)
    is_moving: bool = False
    target_node: Optional[int] = None
    arrival_time: Optional[float] = None
    current_lane: Optional[Tuple[int, int]] = None
    start_time: float = 0.0
    waiting_time: float = 0.0
    trajectory: List[Tuple[float, int]] = field(default_factory=list)

class SimulationMetrics:
    def __init__(self):
        self.total_robots: int = 0
        self.completed_robots: int = 0
        self.success_rate: float = 0.0
        self.total_travel_time: float = 0.0
        self.avg_travel_time: float = 0.0
        self.avg_waiting_time: float = 0.0
        self.makespan: float = 0.0
        self.throughput: float = 0.0
        self.num_replans: int = 0
        self.num_deadlocks_detected: int = 0
        self.num_deadlocks_resolved: int = 0
        self.collisions_prevented: int = 0
        self.lane_utilization: Dict[str, int] = {}