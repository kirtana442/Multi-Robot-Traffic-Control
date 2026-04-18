# test_suite.py
import networkx as nx
import os
from models import Lane, LaneType, Robot
from coordinator import Coordinator
from simulator import Simulator


def create_warehouse_map() -> nx.DiGraph:
    """Same as original sample map (for baseline)."""
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


def create_stress_map() -> nx.DiGraph:
    """Larger 20-node warehouse-style map with many intersections,
    narrow corridors, human zones and crossing paths.
    Designed for high-density (12+ robots) stress testing:
    - congestion, replanning, deadlock, reservation conflicts, safety zones.
    """
    G = nx.DiGraph()
    pos = {i: (i % 5 * 10, i // 5 * 10) for i in range(20)}
    for n, p in pos.items():
        G.add_node(n, pos=p)

    # 38 directed lanes (bidirectional where natural)
    lanes_data = [
        # Row 0 horizontal
        (0,1,"S01",5.0,5,LaneType.NORMAL), (1,0,"S10",5.0,5,LaneType.NORMAL),
        (1,2,"S12",5.0,5,LaneType.NORMAL), (2,1,"S21",5.0,5,LaneType.NORMAL),
        (2,3,"S23",5.0,5,LaneType.NORMAL), (3,2,"S32",5.0,5,LaneType.NORMAL),
        (3,4,"S34",5.0,5,LaneType.NORMAL), (4,3,"S43",5.0,5,LaneType.NORMAL),
        # Row 1 horizontal
        (5,6,"S56",3.0,3,LaneType.NARROW), (6,5,"S65",3.0,3,LaneType.NARROW),
        (6,7,"S67",5.0,5,LaneType.NORMAL), (7,6,"S76",5.0,5,LaneType.NORMAL),
        (7,8,"S78",5.0,5,LaneType.NORMAL), (8,7,"S87",5.0,5,LaneType.NORMAL),
        (8,9,"S89",4.0,4,LaneType.INTERSECTION), (9,8,"S98",4.0,4,LaneType.INTERSECTION),
        # Row 2 horizontal
        (10,11,"S1011",5.0,5,LaneType.NORMAL), (11,10,"S1110",5.0,5,LaneType.NORMAL),
        (11,12,"S1112",3.0,3,LaneType.NARROW), (12,11,"S1211",3.0,3,LaneType.NARROW),
        (12,13,"S1213",5.0,5,LaneType.NORMAL), (13,12,"S1312",5.0,5,LaneType.NORMAL),
        (13,14,"S1314",5.0,5,LaneType.NORMAL), (14,13,"S1413",5.0,5,LaneType.NORMAL),
        # Row 3 horizontal
        (15,16,"S1516",4.0,2,LaneType.HUMAN_ZONE), (16,15,"S1615",4.0,2,LaneType.HUMAN_ZONE),
        (16,17,"S1617",5.0,5,LaneType.NORMAL), (17,16,"S1716",5.0,5,LaneType.NORMAL),
        (17,18,"S1718",5.0,5,LaneType.NORMAL), (18,17,"S1817",5.0,5,LaneType.NORMAL),
        (18,19,"S1819",4.0,4,LaneType.INTERSECTION), (19,18,"S1918",4.0,4,LaneType.INTERSECTION),
        # Vertical connections (cross-layer)
        (0,5,"V05",4.0,4,LaneType.INTERSECTION), (5,0,"V50",4.0,4,LaneType.INTERSECTION),
        (1,6,"V16",4.0,4,LaneType.INTERSECTION), (6,1,"V61",4.0,4,LaneType.INTERSECTION),
        (2,7,"V27",3.0,3,LaneType.NARROW), (7,2,"V72",3.0,3,LaneType.NARROW),
        (3,8,"V38",5.0,5,LaneType.NORMAL), (8,3,"V83",5.0,5,LaneType.NORMAL),
        (4,9,"V49",4.0,4,LaneType.INTERSECTION), (9,4,"V94",4.0,4,LaneType.INTERSECTION),
        (5,10,"V510",5.0,5,LaneType.NORMAL), (10,5,"V105",5.0,5,LaneType.NORMAL),
        (6,11,"V611",4.0,2,LaneType.HUMAN_ZONE), (11,6,"V116",4.0,2,LaneType.HUMAN_ZONE),
        (7,12,"V712",5.0,5,LaneType.NORMAL), (12,7,"V127",5.0,5,LaneType.NORMAL),
        (8,13,"V813",4.0,4,LaneType.INTERSECTION), (13,8,"V138",4.0,4,LaneType.INTERSECTION),
        (9,14,"V914",5.0,5,LaneType.NORMAL), (14,9,"V149",5.0,5,LaneType.NORMAL),
        (10,15,"V1015",4.0,4,LaneType.INTERSECTION), (15,10,"V1510",4.0,4,LaneType.INTERSECTION),
        (11,16,"V1116",5.0,5,LaneType.NORMAL), (16,11,"V1611",5.0,5,LaneType.NORMAL),
        (12,17,"V1217",3.0,3,LaneType.NARROW), (17,12,"V1712",3.0,3,LaneType.NARROW),
        (13,18,"V1318",5.0,5,LaneType.NORMAL), (18,13,"V1813",5.0,5,LaneType.NORMAL),
        (14,19,"V1419",4.0,4,LaneType.INTERSECTION), (19,14,"V1914",4.0,4,LaneType.INTERSECTION),
    ]

    for u, v, lid, ms, sl, lt in lanes_data:
        lane = Lane(lane_id=lid, max_speed=ms, safety_level=sl, lane_type=lt, length=10.0)
        G.add_edge(u, v, lane=lane, length=10.0)
    return G


if __name__ == "__main__":
    print("Running comprehensive test suite (2026 LMAPF-inspired)...\n")

    tests = [
        # 1. Basic – no conflicts (3 robots)
        {"name": "basic_no_conflict",
         "map": create_warehouse_map(),
         "robots": [Robot(i, s, g, s, i) for i, (s, g) in enumerate([(0, 3), (4, 7), (8, 9)])]},

        # 2. High congestion (8 robots pushing narrow lanes → many replans)
        {"name": "high_congestion",
         "map": create_warehouse_map(),
         "robots": [Robot(i, s, g, s, i % 3) for i, (s, g) in enumerate(
             [(0, 3), (1, 3), (2, 3), (4, 9), (5, 9), (6, 9), (0, 9), (4, 3)])]},

        # 3. Deadlock-prone crossing (classic opposing traffic)
        {"name": "deadlock_crossing",
         "map": create_warehouse_map(),
         "robots": [Robot(i, s, g, s, i) for i, (s, g) in enumerate(
             [(0, 7), (7, 0), (1, 6), (6, 1), (2, 5), (5, 2)])]},

        # 4. Safety-critical (human zones + intersections forced)
        {"name": "safety_critical",
         "map": create_warehouse_map(),
         "robots": [Robot(i, s, g, s, i) for i, (s, g) in enumerate(
             [(6, 9), (9, 6), (5, 8), (8, 5)])]},

        # 5. High-density stress (12 robots on 20-node map)
        {"name": "high_density_12",
         "map": create_stress_map(),
         "robots": [Robot(i, i % 15, (i + 5) % 15, i % 15, i) for i in range(12)]},

        # 6. Reservation conflicts (multiple robots fighting same lanes)
        {"name": "reservation_conflicts",
         "map": create_warehouse_map(),
         "robots": [Robot(i, s, g, s, i) for i, (s, g) in enumerate(
             [(0, 3), (0, 3), (0, 3), (4, 7), (4, 7)])]},
    ]

    for t in tests:
        print(f"\nRunning test: {t['name']}")
        G = t["map"]
        robots = t["robots"]
        coordinator = Coordinator(G)
        sim = Simulator(G, robots, coordinator, test_name=t["name"])
        sim.run()
        print(f"   Completed with {sim.metrics.success_rate:.1f}% success")

    print("\nAll tests finished!")
    print("   Results saved in ./test_results/ folder")
    print("   Each subfolder contains:")
    print("     • metrics.json")
    print("     • trajectories.json")
    print("     • lane_heatmap.png")