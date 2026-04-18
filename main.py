# main.py
import networkx as nx
from models import Lane, LaneType, Robot
from coordinator import Coordinator
from simulator import Simulator
from typing import List

def create_sample_map() -> nx.DiGraph:
    """Small warehouse-style map (10 nodes, mix of directed/undirected lanes)."""
    G = nx.DiGraph()
    pos = {0: (0, 0), 1: (10, 0), 2: (20, 0), 3: (30, 0),
           4: (0, 10), 5: (10, 10), 6: (20, 10), 7: (30, 10),
           8: (15, 20), 9: (25, 20)}
    for n, p in pos.items():
        G.add_node(n, pos=p)

    lanes_data = [
        (0,1,"L01",5.0,5,LaneType.NORMAL), (1,0,"L10",5.0,5,LaneType.NORMAL),
        (1,2,"L12",5.0,5,LaneType.NORMAL), (2,1,"L21",5.0,5,LaneType.NORMAL),
        (2,3,"L23",5.0,5,LaneType.NORMAL), (3,2,"L32",5.0,5,LaneType.NORMAL),
        (4,5,"L45",5.0,5,LaneType.NORMAL), (5,4,"L54",5.0,5,LaneType.NORMAL),
        (5,6,"L56",3.0,3,LaneType.NARROW), (6,5,"L65",3.0,3,LaneType.NARROW),
        (6,7,"L67",5.0,5,LaneType.NORMAL), (7,6,"L76",5.0,5,LaneType.NORMAL),
        (0,4,"V04",4.0,4,LaneType.INTERSECTION), (4,0,"V40",4.0,4,LaneType.INTERSECTION),
        (1,5,"V15",4.0,4,LaneType.INTERSECTION), (5,1,"V51",4.0,4,LaneType.INTERSECTION),
        (2,6,"V26",4.0,4,LaneType.INTERSECTION), (6,2,"V62",4.0,4,LaneType.INTERSECTION),
        (3,7,"V37",4.0,4,LaneType.INTERSECTION), (7,3,"V73",4.0,4,LaneType.INTERSECTION),
        (5,8,"L58",4.0,4,LaneType.NORMAL),
        (6,9,"L69",4.0,2,LaneType.HUMAN_ZONE), (9,6,"L96",4.0,2,LaneType.HUMAN_ZONE),
    ]
    for u, v, lid, ms, sl, lt in lanes_data:
        lane = Lane(lane_id=lid, max_speed=ms, safety_level=sl, lane_type=lt, length=10.0)
        G.add_edge(u, v, lane=lane, length=10.0)
    return G

def create_sample_robots() -> List[Robot]:
    """8 robots with crossing paths to trigger coordination, conflicts, and replans."""
    data = [(0,3,0), (4,7,1), (3,0,2), (7,4,3), (0,8,4), (4,9,5), (2,6,6), (6,3,7)]
    return [Robot(robot_id=i, start_node=s, goal_node=g, current_node=s, priority=p, start_time=0.0)
            for i, (s, g, p) in enumerate(data)]

if __name__ == "__main__":
    print("Lane-Aware Multi-Robot Traffic Control Simulator (Hackathon Prototype)")
    G = create_sample_map()
    robots = create_sample_robots()
    coordinator = Coordinator(G)
    simulator = Simulator(G, robots, coordinator)
    simulator.run()

    m = simulator.metrics
    print("\n=== RESULTS SUMMARY ===")
    print(f"Total robots: {m.total_robots}")
    print(f"Completed robots: {m.completed_robots}")
    print(f"Success rate: {m.success_rate:.1f}%")
    print(f"Makespan: {m.makespan:.1f} time units")
    print(f"Average travel time: {m.avg_travel_time:.1f}")
    print(f"Average waiting time: {m.avg_waiting_time:.1f}")
    print(f"Throughput: {m.throughput:.2f} robots/unit time")
    print(f"Replans performed: {m.num_replans}")
    print(f"Deadlocks detected/resolved: {m.num_deadlocks_detected}/{m.num_deadlocks_resolved}")
    print(f"Collision events prevented: {m.collisions_prevented}")
    print("Lane utilization hotspots (top 5):")
    for lid, usage in sorted(m.lane_utilization.items(), key=lambda x: -x[1])[:5]:
        print(f"  {lid}: {usage} traversals")
    print("\nSimulation finished. Check lane_heatmap.png and console output.")